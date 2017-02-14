/***************************************************************************
 *  rosplan_interface_behaviorengine.cpp - Call skills on Lua BE from ROSPlan
 *
 *  Created: Fri Feb 10 13:40:14 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <fawkes_msgs/ExecSkillAction.h>

#include <map>
#include <string>
#include <regex>

#define REGEX_PARAM "\\?\\(([a-zA-Z0-9_-]+)((\\|/([^/]+)/([^/]+)/)*)\\)(s|S|i|f|y|Y)"

typedef enum {
	ACTION_ENABLED,
	ACTION_ACHIEVED,
	ACTION_FAILED
} ActionStatus;

static std::map<ActionStatus, std::string> ActionStatus2String
	{ {ACTION_ENABLED, "action enabled"},
	  {ACTION_ACHIEVED, "action achieved"},
	  {ACTION_FAILED, "action failed"} };

class ROSPlanInterfaceBehaviorEngine {
	typedef actionlib::SimpleActionClient<fawkes_msgs::ExecSkillAction> SkillerClient;

 public:
	ROSPlanInterfaceBehaviorEngine(ros::NodeHandle &n)
		: n(n),
		  skiller_client_(n, "skiller", /* spin thread */ false)
	{
		sub_action_dispatch_ = n.subscribe("kcl_rosplan/action_dispatch", 10,
		                                   &ROSPlanInterfaceBehaviorEngine::cb_action_dispatch, this);
		pub_action_feedback_ =
			n.advertise<rosplan_dispatch_msgs::ActionFeedback>("kcl_rosplan/action_feedback", 10, true);

		svc_update_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("kcl_rosplan/update_knowledge_base",
			                                                                /* persistent */ true);
		ROS_INFO("Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
		
		get_action_specs();
		get_action_mappings();
	}

	void
	get_action_spec(const std::string &name)
	{
		ros::ServiceClient opdetail_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>
			  ("kcl_rosplan/get_domain_operator_details");
		if (! opdetail_client.waitForExistence(ros::Duration(10))) {
			ROS_ERROR("Could not discover get_domain_operator_details service "
			          "(for action spec '%s')", name.c_str());
			return;
		}
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = name;
		if(opdetail_client.call(srv)) {
			std::set<std::string> reqp;
			for (const auto &p : srv.response.op.formula.typed_parameters) {
				reqp.insert(p.key);
			}
			specs_[name] = { srv.response.op.formula, srv.response.op, reqp };
		} else {
			ROS_ERROR("Could not get spec for operator %s", name.c_str());
			return;
		}
	}

	void
	get_action_specs()
	{
		ros::ServiceClient oplist_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>
			  ("kcl_rosplan/get_domain_operators");
		if (! oplist_client.waitForExistence(ros::Duration(120))) {
			ROS_ERROR("Could not retrieve action specs from ROSPlan");
			return;
		}
		
		rosplan_knowledge_msgs::GetDomainOperatorService oplist_srv;
		if (oplist_client.call(oplist_srv)) {
			for (const auto &op : oplist_srv.response.operators) {
				ROS_INFO("Retrieving action spec for %s", op.name.c_str());
				get_action_spec(op.name);
			}
		} else {
			ROS_ERROR("Failed to get list of operators");
		}

		get_predicates();
	}

	void
	get_action_mappings()
	{
		ros::NodeHandle privn("~");
		for (const auto &sp : specs_) {
			const std::string &name = sp.first;
			const RPActionSpec &spec = sp.second;
			std::string value;
			if (privn.getParam("action_mappings/" + name, value)) {
				std::regex re(REGEX_PARAM);
				std::smatch m;
				bool ok = true;
				std::string s = value;
				while (std::regex_search(s, m, re)) {
					if (spec.required_params.find(m[1]) == spec.required_params.end()) {
						ROS_ERROR("Invalid argument %s for action %s", m[0].str().c_str(), name.c_str());
						ok = false;
						break;
					}
					s = m.suffix();
				}
				if (ok) {
					ROS_INFO("Action '%s' maps to '%s'", name.c_str(), value.c_str());
					mappings_[name] = value;
				} else {
					ROS_WARN("Ignoring action %s", name.c_str());
				}
			} else {
				ROS_ERROR("Failed to get mapping for action %s", name.c_str());
			}
		}
	}
	
	void collect_predicates(std::set<std::string> &predicate_names,
	                        const std::vector<rosplan_knowledge_msgs::DomainFormula> &df)
	{
		std::for_each(df.begin(), df.end(),
		              [&predicate_names](auto &f) { predicate_names.insert(f.name); });
	}

	void
	get_predicates()
	{
		std::set<std::string> predicate_names;
		for (const auto &sp : specs_) {
			const RPActionSpec &s = sp.second;
			collect_predicates(predicate_names, s.op.at_start_add_effects);
			collect_predicates(predicate_names, s.op.at_start_del_effects);
			collect_predicates(predicate_names, s.op.at_end_add_effects);
			collect_predicates(predicate_names, s.op.at_end_del_effects);
			collect_predicates(predicate_names, s.op.at_start_simple_condition);
			collect_predicates(predicate_names, s.op.over_all_simple_condition);
			collect_predicates(predicate_names, s.op.at_end_simple_condition);
			collect_predicates(predicate_names, s.op.at_start_neg_condition);
			collect_predicates(predicate_names, s.op.over_all_neg_condition);
			collect_predicates(predicate_names, s.op.at_end_neg_condition);
		}

		// fetch and store predicate details
		ros::service::waitForService("kcl_rosplan/get_domain_predicate_details",ros::Duration(20));
		ros::ServiceClient pred_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>
			  ("kcl_rosplan/get_domain_predicate_details", /* persistent */ true);
		if (! pred_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("No service provider for get_domain_predicate_details");
			return;
		}

		for (const auto &pn : predicate_names) {
			ROS_INFO("Relevant predicate: %s", pn.c_str());
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService pred_srv;
			pred_srv.request.name = pn;
			if(pred_client.call(pred_srv)) {
				predicates_[pn] = pred_srv.response.predicate;
			} else {
				ROS_ERROR("Failed to get predicate details for %s", pn.c_str());
				return;
			}
		}
	}

	void
	send_action_fb(int id, ActionStatus status)
	{
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = id;
		fb.status = ActionStatus2String[status];
		pub_action_feedback_.publish(fb);
	}
	
	void
	cb_action_dispatch(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		const std::string &name(msg->name);

		if (specs_.find(name) == specs_.end()) {
			ROS_WARN("Unknown action %s called, failing", name.c_str());
			send_action_fb(msg->action_id, ACTION_FAILED);
			return;
		}

		std::set<std::string> params;
		std::for_each(msg->parameters.begin(), msg->parameters.end(),
		              [&params](const auto &kv) { params.insert(kv.key); });

		std::vector<std::string> diff;
		std::set_difference(specs_[name].required_params.begin(), specs_[name].required_params.end(),
		                    params.begin(), params.end(), std::inserter(diff, diff.begin()));

		if (! diff.empty()) {
			std::string diff_s;
			std::for_each(diff.begin(), diff.end(), [&diff_s](const auto &s) { diff_s += " " + s; });
			ROS_WARN("Invalid call to %s (invalid or missing args %s), failing",
			         name.c_str(), diff_s.c_str());
			send_action_fb(msg->action_id, ACTION_FAILED);
			return;
		}

		std::string param_str;
		std::for_each(msg->parameters.begin(), msg->parameters.end(),
		              [&param_str](const auto &kv) { param_str += " " + kv.key + "=" + kv.value; });

		std::string skill_string = map_skill(name, msg->parameters);

		if (skill_string.empty()) {
			ROS_ERROR("Failed to translate (%s%s)",
			          name.c_str(), param_str.c_str());
			send_action_fb(msg->action_id, ACTION_FAILED);
			return;
		}
			
		ROS_INFO("Executing (%s%s) -> %s", name.c_str(), param_str.c_str(), skill_string.c_str());

		cur_msg_ = *msg;
		cur_skill_string_ = skill_string;

		start_execute(skill_string);
	}

	std::string
	map_skill(const std::string &name, const std::vector<diagnostic_msgs::KeyValue> &params)
	{
		std::string rv;
		std::string remainder = mappings_[name];

		std::regex re(REGEX_PARAM);
		std::smatch m;
		bool ok = true;
		while (std::regex_search(remainder, m, re)) {
			ROS_DEBUG("Variable match: %s", m[0].str().c_str());
			bool found = false;
			for (const auto &p : params) {
				std::string value = p.value;
				if (p.key == m[1].str()) {
					found = true;
					rv += m.prefix();

					if (! m[2].str().empty()) {
						std::string rstr = m[2].str();
						std::list<std::string> rlst;
						std::string::size_type rpos = 0, fpos = 0;
						while ((fpos = rstr.find('|', rpos)) != std::string::npos) {
							std::string substr = rstr.substr(rpos, fpos-rpos);
							if (! substr.empty())  rlst.push_back(substr);
							rpos = fpos + 1;
						}
						rstr = rstr.substr(rpos);
						if (! rstr.empty())  rlst.push_back(rstr);
						
						for (const auto &r : rlst) {
							if (r.size() > 2 && r[0] == '/' && r[r.size()-1] == '/') {
								std::string::size_type slash_pos = r.find('/', 1);
								if (slash_pos != std::string::npos && slash_pos < (r.size() - 1)) {
									std::string r_match = r.substr(1, slash_pos - 1);
									std::string r_repl  = r.substr(slash_pos + 1, (r.size() - slash_pos - 2));
									ROS_DEBUG("Replace '%s' by '%s' in '%s'", r_match.c_str(), r_repl.c_str(), value.c_str());
									std::regex user_regex(r_match);
									value = std::regex_replace(value, user_regex, r_repl);
								} else {
									ROS_WARN("Regex '%s' missing mid slash, ignoring", r.c_str());
								}
							} else {
								ROS_WARN("Regex '%s' missing start/end slashes, ignoring", r.c_str());
							}
						}
					}

					switch (m[6].str()[0]) {
					case 's': rv += "\"" + value + "\""; break;
					case 'S':
						{
							std::string uc = value;
							std::transform(uc.begin(), uc.end(), uc.begin(), ::toupper);
							rv += "\"" + uc + "\"";
						}
						break;
					case 'y': rv += value; break;
					case 'Y':
						{
							std::string uc = value;
							std::transform(uc.begin(), uc.end(), uc.begin(), ::toupper);
							rv += uc;
						}
						break;
					case 'i': rv += std::to_string(std::stol(value)); break;
					case 'f': rv += std::to_string(std::stod(value)); break;
					}
					break;
				}
			}
			if (! found) {
				ROS_ERROR("No value for parameter '%s' of action '%s' given",
				          m[1].str().c_str(), name.c_str());
				return "";
			}

			remainder = m.suffix();
		}
		rv += remainder;
		
		return rv;
	}

	// Called once when the goal completes
	void execute_done_cb(const actionlib::SimpleClientGoalState& state,
	                     const fawkes_msgs::ExecSkillResultConstPtr& result)
	{
		ROS_INFO("Finished '%s' in state [%s]",
		         cur_skill_string_.c_str(), state.toString().c_str());

		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			send_predicate_update(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE,
			                      specs_[cur_msg_.name].op.at_end_del_effects);
			send_predicate_update(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE,
			                      specs_[cur_msg_.name].op.at_end_add_effects);

			send_action_fb(cur_msg_.action_id, ACTION_ACHIEVED);
			cur_skill_string_ = "";
			cur_msg_.action_id = 0;
		}
		else if (state == actionlib::SimpleClientGoalState::ABORTED ||
		           state == actionlib::SimpleClientGoalState::REJECTED)
		{
			ROS_WARN("Execution of '%s' failed: %s", cur_skill_string_.c_str(),
			         result->errmsg.c_str());
			send_action_fb(cur_msg_.action_id, ACTION_FAILED);
			cur_skill_string_ = "";
			cur_msg_.action_id = 0;
		}	
	}

	// Called once when the goal becomes active
	void execute_active_cb()
	{
		ROS_INFO("Goal for '%s' just went active", cur_skill_string_.c_str());
		send_action_fb(cur_msg_.action_id, ACTION_ENABLED);

		send_predicate_update(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE,
		                      specs_[cur_msg_.name].op.at_start_del_effects);
		send_predicate_update(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE,
		                      specs_[cur_msg_.name].op.at_start_add_effects);
	}

	void
	send_predicate_update(int op,
	                      const std::vector<rosplan_knowledge_msgs::DomainFormula> &dfv)
	{
		for (const auto &df : dfv) {
			rosplan_knowledge_msgs::KnowledgeUpdateService srv;
			srv.request.update_type = op;
			srv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			srv.request.knowledge.attribute_name = df.name;
			srv.request.knowledge.values.clear();

			for (const auto &p : df.typed_parameters) {
				diagnostic_msgs::KeyValue pair;
				pair.key = p.key;
				for (const auto &mp : cur_msg_.parameters) {
					if (mp.key == p.key) {
						pair.value = mp.value;
						break;
					}
				}
				srv.request.knowledge.values.push_back(pair);
			}

			std::string param_str;
			std::for_each(srv.request.knowledge.values.begin(), srv.request.knowledge.values.end(),
			              [&param_str](const auto &kv) { param_str += " " + kv.key + "=" + kv.value; });

			ROS_INFO("%s (%s%s)",
			         (op == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE)
			         ? "Asserting" : "Retracting",
			         srv.request.knowledge.attribute_name.c_str(),
			         param_str.c_str());
			if( ! svc_update_knowledge_.call(srv)) {
				ROS_INFO("Failed to update predicate %s", df.name.c_str());
			}
		}
	}

	void start_execute(const std::string &skill_string)
	{
		fawkes_msgs::ExecSkillGoal goal;
		goal.skillstring = skill_string;
		skiller_client_.sendGoal(goal,
		                         boost::bind(&ROSPlanInterfaceBehaviorEngine::execute_done_cb, this, _1, _2),
		                         boost::bind(&ROSPlanInterfaceBehaviorEngine::execute_active_cb, this));
	}
	
 private:
	ros::NodeHandle    n;

	ros::Subscriber    sub_action_dispatch_;
	ros::Publisher     pub_action_feedback_;
	ros::ServiceClient svc_update_knowledge_;

	SkillerClient      skiller_client_;

	struct RPActionSpec {
		rosplan_knowledge_msgs::DomainFormula  params;
		rosplan_knowledge_msgs::DomainOperator op;
		std::set<std::string> required_params;
	};
	std::map<std::string, RPActionSpec> specs_;
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates_;

	std::map<std::string, std::string>  mappings_;

	rosplan_dispatch_msgs::ActionDispatch cur_msg_;
	std::string                           cur_skill_string_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_interface_behaviorengine");

	ros::NodeHandle n;

	ROSPlanInterfaceBehaviorEngine rosplan_be(n);
	
  ros::spin();
  
	return 0;
}

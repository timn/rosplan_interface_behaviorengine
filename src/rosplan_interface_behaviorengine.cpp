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
		
		get_action_specs();
	}

	void
	get_action_spec(const std::string &name)
	{
		ros::ServiceClient opdetail_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>
			  ("kcl_rosplan/get_domain_operator_details");
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
			  ("/kcl_rosplan/get_domain_predicate_details", /* persistent */ true);

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
		ROS_INFO("Executing (%s%s)", name.c_str(), param_str.c_str());
		send_action_fb(msg->action_id, ACTION_ACHIEVED);
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

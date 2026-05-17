#ifndef __GRAPH_PUBLISHER_HPP__
#define __GRAPH_PUBLISHER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <set>
#include <vector>
#include <utility>
#include <fstream>

#include "System.h"
#include "MapPoint.h"
#include "KeyFrame.h"

class GraphPublisher
{
public:
    GraphPublisher(rclcpp::Node *node) : node_(node)
    {
        covisibility_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("orb_slam3/covisibility_graph", 10);
        essential_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("orb_slam3/essential_graph", 10);
    }

    Eigen::Vector3f TransformPointForOutput(const Eigen::Vector3f &point) const
    {
        return Eigen::Vector3f(point(2), -point(0), -point(1));
    }

    void PublishGraphs(ORB_SLAM3::System *pSLAM, const builtin_interfaces::msg::Time &stamp)
    {
        if (covisibility_pub_->get_subscription_count() == 0 && essential_pub_->get_subscription_count() == 0)
        {
            return;
        }

        // Rate limit: Only publish every 10 frames to reduce overhead and race risks
        static int frame_count = 0;
        if (++frame_count % 10 != 0) return;

        std::vector<ORB_SLAM3::MapPoint *> all_map_points = pSLAM->GetAllMapPoints();
        if (all_map_points.empty()) return;

        std::set<ORB_SLAM3::KeyFrame *> keyframes;
        for (auto pMP : all_map_points)
        {
            if (pMP && !pMP->isBad())
            {
                auto obs = pMP->GetObservations();
                for (auto const &item : obs)
                {
                    if (item.first && !item.first->isBad())
                    {
                        keyframes.insert(item.first);
                    }
                }
            }
        }

        if (keyframes.empty())
            return;

        visualization_msgs::msg::Marker cov_msg;
        cov_msg.header.frame_id = "root";
        cov_msg.header.stamp = stamp;
        cov_msg.ns = "covisibility_graph";
        cov_msg.id = 0;
        cov_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        cov_msg.action = visualization_msgs::msg::Marker::ADD;
        cov_msg.scale.x = 0.01;
        cov_msg.color.r = 0.0;
        cov_msg.color.g = 1.0;
        cov_msg.color.b = 0.0;
        cov_msg.color.a = 0.5;

        visualization_msgs::msg::Marker ess_msg;
        ess_msg.header.frame_id = "root";
        ess_msg.header.stamp = stamp;
        ess_msg.ns = "essential_graph";
        ess_msg.id = 0;
        ess_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        ess_msg.action = visualization_msgs::msg::Marker::ADD;
        ess_msg.scale.x = 0.02;
        ess_msg.color.r = 1.0;
        ess_msg.color.g = 0.0;
        ess_msg.color.b = 0.0;
        ess_msg.color.a = 1.0;

        std::set<std::pair<long unsigned int, long unsigned int>> cov_edges;
        std::set<std::pair<long unsigned int, long unsigned int>> ess_edges;

        for (auto pKF : keyframes)
        {
            if (!pKF || pKF->isBad())
                continue;

            // CRITICAL: Check if the pose is valid BEFORE calling Sophus methods like GetPoseInverse()
            // In many ORB-SLAM3 forks, GetPose() returns the raw SE3 object.
            Sophus::SE3f Tcw = pKF->GetPose();
            Eigen::Vector3f t = Tcw.translation();
            if (!std::isfinite(t.x()) || !std::isfinite(t.y()) || !std::isfinite(t.z()))
                continue;

            Sophus::SE3f T1 = pKF->GetPoseInverse();
            Eigen::Vector3f trans1 = T1.translation();
            if (!std::isfinite(trans1.x()) || !std::isfinite(trans1.y()) || !std::isfinite(trans1.z()))
                continue;

            Eigen::Vector3f pw1 = TransformPointForOutput(trans1);
            geometry_msgs::msg::Point p1;
            p1.x = pw1.x();
            p1.y = pw1.y();
            p1.z = pw1.z();

            auto add_edge = [&](ORB_SLAM3::KeyFrame *kf2, visualization_msgs::msg::Marker &msg, std::set<std::pair<long unsigned int, long unsigned int>> &edge_set)
            {
                if (!kf2 || kf2->isBad())
                    return;

                Sophus::SE3f Tcw2 = kf2->GetPose();
                if (!std::isfinite(Tcw2.translation().x())) return;

                long unsigned int id1 = pKF->mnId;
                long unsigned int id2 = kf2->mnId;
                if (id1 > id2)
                    std::swap(id1, id2);
                if (edge_set.count({id1, id2}))
                    return;
                edge_set.insert({id1, id2});

                Sophus::SE3f T2 = kf2->GetPoseInverse();
                Eigen::Vector3f trans2 = T2.translation();
                
                if (!std::isfinite(trans2.x()) || !std::isfinite(trans2.y()) || !std::isfinite(trans2.z()))
                    return;

                Eigen::Vector3f pw2 = TransformPointForOutput(trans2);
                geometry_msgs::msg::Point p2;
                p2.x = pw2.x();
                p2.y = pw2.y();
                p2.z = pw2.z();
                msg.points.push_back(p1);
                msg.points.push_back(p2);
            };

            // Essential Graph: Parent
            auto pParent = pKF->GetParent();
            if (pParent)
            {
                add_edge(pParent, ess_msg, ess_edges);
            }

            // Essential Graph: Loop edges
            auto loop_edges = pKF->GetLoopEdges();
            for (auto pLoopKF : loop_edges)
            {
                add_edge(pLoopKF, ess_msg, ess_edges);
            }

            // Covisibility edges
            auto covisible_kfs = pKF->GetVectorCovisibleKeyFrames();
            for (auto pCovKF : covisible_kfs)
            {
                // All covisibility edges go to covisibility graph
                add_edge(pCovKF, cov_msg, cov_edges);

                // Strong covisibility edges (weight > 100) go to Essential Graph
                if (pKF->GetWeight(pCovKF) >= 100)
                {
                    add_edge(pCovKF, ess_msg, ess_edges);
                }
            }
        }

        if (covisibility_pub_->get_subscription_count() > 0)
            covisibility_pub_->publish(cov_msg);
        if (essential_pub_->get_subscription_count() > 0)
            essential_pub_->publish(ess_msg);
    }
    void SaveGraphs(ORB_SLAM3::System *pSLAM, const std::string &cov_file, const std::string &ess_file)
    {
        std::vector<ORB_SLAM3::MapPoint *> all_map_points = pSLAM->GetAllMapPoints();
        std::set<ORB_SLAM3::KeyFrame *> keyframes;

        for (auto pMP : all_map_points)
        {
            if (pMP && !pMP->isBad())
            {
                auto obs = pMP->GetObservations();
                for (auto const &item : obs)
                {
                    if (item.first && !item.first->isBad())
                    {
                        keyframes.insert(item.first);
                    }
                }
            }
        }

        if (keyframes.empty())
            return;

        std::ofstream cov_out(cov_file);
        std::ofstream ess_out(ess_file);

        std::set<std::pair<long unsigned int, long unsigned int>> cov_edges;
        std::set<std::pair<long unsigned int, long unsigned int>> ess_edges;

        for (auto pKF : keyframes)
        {
            if (!pKF || pKF->isBad())
                continue;

            Sophus::SE3f T1 = pKF->GetPoseInverse();
            Eigen::Vector3f trans1 = T1.translation();
            
            if (!std::isfinite(trans1.x()) || !std::isfinite(trans1.y()) || !std::isfinite(trans1.z()))
                continue;

            Eigen::Vector3f pw1 = TransformPointForOutput(trans1);

            auto add_edge = [&](ORB_SLAM3::KeyFrame *kf2, std::ofstream &out, std::set<std::pair<long unsigned int, long unsigned int>> &edge_set)
            {
                if (!kf2 || kf2->isBad())
                    return;
                long unsigned int id1 = pKF->mnId;
                long unsigned int id2 = kf2->mnId;
                if (id1 > id2)
                    std::swap(id1, id2);
                if (edge_set.count({id1, id2}))
                    return;
                edge_set.insert({id1, id2});

                Sophus::SE3f T2 = kf2->GetPoseInverse();
                Eigen::Vector3f trans2 = T2.translation();
                
                if (!std::isfinite(trans2.x()) || !std::isfinite(trans2.y()) || !std::isfinite(trans2.z()))
                    return;

                Eigen::Vector3f pw2 = TransformPointForOutput(trans2);
                out << pw1.x() << " " << pw1.y() << " " << pw1.z() << " "
                    << pw2.x() << " " << pw2.y() << " " << pw2.z() << "\n";
            };

            // Essential Graph: Parent
            auto pParent = pKF->GetParent();
            if (pParent)
            {
                add_edge(pParent, ess_out, ess_edges);
            }

            // Essential Graph: Loop edges
            auto loop_edges = pKF->GetLoopEdges();
            for (auto pLoopKF : loop_edges)
            {
                add_edge(pLoopKF, ess_out, ess_edges);
            }

            // Covisibility edges
            auto covisible_kfs = pKF->GetVectorCovisibleKeyFrames();
            for (auto pCovKF : covisible_kfs)
            {
                add_edge(pCovKF, cov_out, cov_edges);

                // Strong covisibility edges (weight > 100) go to Essential Graph
                if (pKF->GetWeight(pCovKF) >= 100)
                {
                    add_edge(pCovKF, ess_out, ess_edges);
                }
            }
        }

        cov_out.close();
        ess_out.close();

        RCLCPP_INFO(node_->get_logger(), "Successfully saved Covisibility Graph with %zu edges to %s", cov_edges.size(), cov_file.c_str());
        RCLCPP_INFO(node_->get_logger(), "Successfully saved Essential Graph with %zu edges to %s", ess_edges.size(), ess_file.c_str());
    }

private:
    rclcpp::Node *node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covisibility_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr essential_pub_;
};

#endif

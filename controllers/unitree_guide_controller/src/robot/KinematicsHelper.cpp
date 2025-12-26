#include "unitree_guide_controller/robot/KinematicsHelper.h"
#include <iostream>
#include <set> // [新增] 需要引入 set 头文件用于去重

Vec34 calculateFeetPosNormalStand(
    const std::vector<std::shared_ptr<RobotLeg>>& legs, 
    const std::vector<double>& stand_pos) 
{
    Vec34 feet_pos;
    feet_pos.setZero();

    if (legs.size() != 4) {
        std::cerr << "[KinematicsHelper] Error: Robot must have 4 legs." << std::endl;
        return feet_pos;
    }

    for (int i = 0; i < 4; ++i) {
        KDL::JntArray q_stand(3);
        q_stand(0) = stand_pos[i * 3 + 0];
        q_stand(1) = stand_pos[i * 3 + 1];
        q_stand(2) = stand_pos[i * 3 + 2];

        KDL::Frame foot_pose = legs[i]->calcPEe2B(q_stand);
        feet_pos.col(i) << foot_pose.p.x(), foot_pose.p.y(), foot_pose.p.z();
    }
    return feet_pos;
}

RobotInertialParams calculateRobotDynamics(
    const KDL::Tree &robot_tree, 
    const std::string &base_name,
    const std::vector<KDL::Chain> &leg_chains, 
    const std::vector<double> &stand_pos)
{
    RobotInertialParams params;
    params.mass = 0.0;
    params.pcb.setZero();
    params.Ib.setZero();

    KDL::RigidBodyInertia total_inertia;
    
    // [关键修改] 使用 set 记录已经累加过的 Segment 名称，防止重复计算
    std::set<std::string> processed_segments;

    // 1. 获取 Base Link (躯干) 的惯量
    auto base_segment_it = robot_tree.getSegment(base_name);
    if(base_segment_it != robot_tree.getSegments().end()){
        total_inertia = total_inertia + base_segment_it->second.segment.getInertia();
        processed_segments.insert(base_segment_it->second.segment.getName()); // 记录 Base
    } else {
        std::cerr << "[KinematicsHelper] Error: Base link '" << base_name << "' not found!" << std::endl;
    }

    // 2. 遍历四条腿
    if(leg_chains.size() != 4) {
         std::cerr << "[KinematicsHelper] Error: 4 leg chains required." << std::endl;
         return params;
    }
    
    int joint_idx_counter = 0;

    for(int i=0; i<4; ++i) {
        KDL::Frame frame_accum = KDL::Frame::Identity(); 
        const KDL::Chain& current_chain = leg_chains[i];
        
        for(unsigned int j=0; j < current_chain.getNrOfSegments(); ++j) {
            auto segment = current_chain.getSegment(j);
            std::string seg_name = segment.getName();

            // 计算当前的变换矩阵 (运动学)
            KDL::Joint joint = segment.getJoint();
            if(joint.getType() == KDL::Joint::None) {
                frame_accum = frame_accum * segment.pose(0);
            } else {
                if(joint_idx_counter < (int)stand_pos.size()) {
                    // 注意：这里简单的索引累加假设 stand_pos 顺序与 chain遍历顺序完全一致
                    // 实际工程中可能需要更严谨的关节名匹配，但对于 Lite3 这种标准构型通常没问题
                    double q = stand_pos[joint_idx_counter];
                    frame_accum = frame_accum * segment.pose(q);
                    joint_idx_counter++;
                }
            }

            // [关键修改] 检查是否已经计算过该连杆
            if (processed_segments.find(seg_name) != processed_segments.end()) {
                // 如果已经在 set 里，说明之前（比如在上一条腿的链里，或者作为 Base）算过了
                // 跳过惯量累加，但必须保留上面的 frame_accum 计算，因为后续连杆的位置依赖于它
                continue; 
            }

            // 没算过 -> 累加惯量，并加入 set
            total_inertia = total_inertia + (frame_accum * segment.getInertia());
            processed_segments.insert(seg_name);
        }
    }

    // 3. 提取结果
    params.mass = total_inertia.getMass();
    
    KDL::Vector com = total_inertia.getCOG();
    params.pcb << com.x(), com.y(), com.z();

    KDL::RotationalInertia I_com = total_inertia.RefPoint(com).getRotationalInertia();
    params.Ib << I_com.data[0], I_com.data[1], I_com.data[2],
                 I_com.data[3], I_com.data[4], I_com.data[5],
                 I_com.data[6], I_com.data[7], I_com.data[8];
                 
    return params;
}
#include "CableJoint.h"

float SurfaceDistance(Vec3 current, Vec3 next)
{
    return 0.0f;
}

Vec3 ComputeNewAttachment(Vec3 current)
{
    return Vec3(0,0,0);
}

void ComputeRestLength_Pinhole(Pinhole_sp pinhole, CableJoint_sp joint)
{
    // Split cable into two new cable
    // With 4? attachments?
}


CableJoint::CableJoint()
{}

CableJoint::~CableJoint()
{

}

CableJointSolver::CableJointSolver()
{

}

void CableJointSolver::AddCableJoint(CableJoint_sp joint)
{
    m_cable_joints.push_back(joint);
}

void CableJointSolver::Timestep(float dt)
{
    for(auto iter=m_cable_joints.begin(); iter != m_cable_joints.end(); ++iter)
    {
        auto joint = *iter;
        // Compute New Attachments
        Vec3 left = joint->getAttachmentLeft();
        Vec3 right = joint->getAttachmentRight();

        Vec3 left_next = ComputeNewAttachment(joint->getAttachmentLeft());
        Vec3 right_next = ComputeNewAttachment(joint->getAttachmentRight());
        
        joint->m_rest_length += SurfaceDistance(left, left_next);
        joint->m_rest_length -= SurfaceDistance(right, right_next);

        left = left_next; 
        right = right_next;

        joint->setAttachementLeft(left);
        joint->setAttachementRight(right);
    }

    for(auto iter=m_cable_joints.begin(); iter!=m_cable_joints.end(); ++iter)
    {
        auto joint = *iter;
        joint->m_d_hat = joint->m_rest_length;
    }

    for(auto iter=m_pinholes.begin(); iter!=m_pinholes.end(); ++iter)
    {
        auto pinhole = *iter;
        auto cable_left = pinhole->cable_left;
        auto cable_right = pinhole->cable_right;

        if(cable_left->m_current_distance > cable_left->m_d_hat)
        {
            float error = cable_left->m_current_distance - cable_left->m_d_hat;
            cable_left->m_rest_length += error;
            cable_right->m_rest_length -= error;
        }

        if(cable_right->m_current_distance > cable_right->m_d_hat)
        {
            float error = cable_right->m_current_distance - cable_right->m_d_hat;
            cable_left->m_rest_length -= error;
            cable_right->m_rest_length += error;
        }

    }

    RunSolver();

}
    
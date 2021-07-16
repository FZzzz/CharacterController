#ifndef _CABLE_JOINT_H_
#define _CABLE_JOINT_H_

#include <memory>
#include <vector>

struct Pinhole;
class CableJoint;
class CableJointSolver;

using Pinhole_sp = std::shared_ptr<Pinhole>;
using CableJoint_sp = std::shared_ptr<CableJoint>;

struct Vec3
{
    Vec3();
    Vec3(float x_in, float y_in, float z_in): 
        x(x_in), y(y_in), z(z_in){};
    
    float x,y,z;
};


float SurfaceDistance(Vec3 current, Vec3 next); //rotate a degree?

class CableJoint
{
public:
    CableJoint();
    ~CableJoint();

    void setAttachementLeft(Vec3);
    void setAttachementRight(Vec3);

    Vec3 getAttachmentLeft();
    Vec3 getAttachmentRight();
    
    float m_rest_length;
    float m_d_hat;
    float m_current_distance;

private:

    Vec3 m_attachment_left;
    Vec3 m_attachment_right;
    
};

struct Pinhole
{
    Pinhole(Vec3 location) : pinhole_location(location){};
    Pinhole(Vec3 location, CableJoint_sp left, CableJoint_sp right):
            pinhole_location(location), cable_left(left), cable_right(right){};

    Vec3 pinhole_location;

    CableJoint_sp cable_left;
    CableJoint_sp cable_right;

};

class CableJointSolver
{

public: 
    CableJointSolver();

    void RunSolver();
    
    void AddCableJoint(CableJoint_sp);
    void AddPinholeLink(Pinhole_sp);

private:
    
    void Timestep(float dt);

    std::vector<CableJoint_sp> m_cable_joints;
    std::vector<Pinhole_sp> m_pinholes;

};

#endif
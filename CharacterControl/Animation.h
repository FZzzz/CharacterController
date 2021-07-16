#ifndef _ANIMATION_H_
#define _ANIMATION_H_

#include "Joint.h"
#include "common.h"
#include <vector>
#include <memory>
#include <unordered_map>

/**
 * [Caution]
 * - Channels stores rotation as degree
 *
**/

// Forward declaration
struct Channel;
struct FrameData;
struct Frame;
class AnimationTransition;
class AnimationState;
class Animation;

using AnimTransition_sp = std::shared_ptr<AnimationTransition>;
using AnimState_sp = std::shared_ptr<AnimationState>;

struct Channel
{
	enum class Channel_Type
	{
		X_TRANSLATION,
		Y_TRANSLATION,
		Z_TRANSLATION,
		X_ROTATION,
		Y_ROTATION,
		Z_ROTATION
	}type;
	float value;
};

struct FrameData
{
	bool movable;
	glm::vec3 translation = glm::vec3(0);
	glm::quat quaternion = glm::quat();
};

struct Frame
{
	uint32_t frame_id;
	std::unordered_map<std::shared_ptr<Joint>, std::vector<Channel>> joint_channel_map;
	std::unordered_map<std::shared_ptr<Joint>, FrameData> joint_framedata_map;
};

// getCurrentFrameData() specific frame -> step() to next frame
// getCurrentFrameTime(); --> for synchronization
// 


/*
*  A state should contain:
*	1. Keyframes
*	2. Transitions to other state
*/
class AnimationState
{
public:
	AnimationState();
	~AnimationState();

	void Pause();
	void Step();

	void NextFrame();

	// setter
	void setFrames(std::vector<Frame>& frames);
	void setCurrentFrame(size_t frame_count);
	/*
	* Set the start frame
	* (Caution!) This will reset the current frame.
	*/
	void setFrameStart(uint32_t frame_start);
	void setFrameTime(float frame_time);
	void setRepeat(bool repeat);

	// getter
	inline const std::vector<Frame>&	getFrames() { return m_frames; };
	inline const Frame&					getCurrentFrame() { return m_frames[m_current_frame]; };
	inline const size_t					getCurrentFrameTime() { return m_current_frame; };
	inline const uint32_t				getFrameStart() { return m_frame_start; };
	inline const size_t					getKeyFrameSize() { return m_frames.size(); };
	inline const std::string			getName() { return m_name; };
	inline const float					getFrameTime() { return m_frame_time; };
	inline bool							isRepeat() { return m_repeat; };
	inline bool							isPausing() { return m_pause; };

	uint32_t m_frame_count;

private:

	// Sub-function of setFrames()
	void PrecomputeFrameData();

	std::vector<Frame>	m_frames;
	float				m_frame_time;
	size_t				m_current_frame;
	uint32_t			m_frame_start;
	std::string			m_name;
	bool				m_pause;
	bool				m_repeat;
	
};

/*
* Transition is a directional relationship between start and end.
* It should have a conditional function registered to determine whether to proceed the transition 
* There are two types of transition provided, the linear transition and bezier transition.
* While the transition is proceeding, call SampleAnimation() to get the character pose.
*/

class AnimationTransition
{
public:
	
	enum ETransitionType
	{
		TRANSITION_LINEAR,
		TRANSITION_BEZIER
	};

	AnimationTransition(AnimState_sp start, AnimState_sp end);
	~AnimationTransition();

	void SampleAnimation(float t);

private:

	void Interpolate();

	AnimState_sp m_start;
	AnimState_sp m_end;
	ETransitionType m_type;

	// The transition rate should be limited between 0 and 1
	float m_transition_progress; 
	

};

/**
* An Animation is composed by multiple States and Transitions.
* It manage/store those information in this class.
* Start with initial state, the animation will be played by following 
* the written/given conditions in Transition class
*/

class Animation
{
public:
	Animation();
	~Animation();

	inline std::vector<AnimState_sp> getStates() { return m_states; };
	inline std::vector<AnimTransition_sp> getTransitions() { return m_transitions; };


private:
	std::vector<AnimState_sp> m_states;
	std::vector<AnimTransition_sp> m_transitions;
	AnimState_sp m_initial_state;
};


#endif
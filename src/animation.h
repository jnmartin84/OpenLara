#ifndef H_ANIMATION
#define H_ANIMATION

#include "utils.h"
#include "format.h"

struct Animation {
    TR::Level       *level;
    TR::Model       *model;
    TR::Animation   *anims;
    int             state;
    float           time, timeMax, delta, dir;
    int             index, prev, next;
    int             frameIndex, framePrev, framesCount;

    TR::AnimFrame   *frameA, *frameB;
    vec3            offset, jump;
    bool            isEnded, isPrepareToNext, flip;

    quat            *overrides;   // left & right arms animation frames
    int             overrideMask;

    Animation() : overrides(NULL) {}

    Animation(TR::Level *level, TR::Model *model) : level(level), model(model), anims(model ? &level->anims[model->animation] : NULL), time(0), delta(0), dir(1.0f),
                                                    index(-1), prev(0), next(0), overrides(NULL), overrideMask(0) {
        if (anims) setAnim(0);
    }

    ~Animation() {
        delete[] overrides;
    }
    
    inline operator TR::Animation* () const { return anims + index; }

    void initOverrides() {
        overrides    = new quat[model->mCount];
        overrideMask = 0;
    }

    void update() {
        if (!isEnded) {
            time += dir * Core::deltaTime;
            isEnded = time <= 0.0f || time >= timeMax;
            time = clamp(time, 0.0f, timeMax - EPS);
        }
        updateInfo();
    }

    int setAnim(int animIndex, int animFrame = 0, bool lerpToNext = true) {
        TR::Animation *anim = anims + animIndex;
        isEnded     = isPrepareToNext = false;
        offset      = jump = vec3(0.0f);
        prev        = index;
        index       = animIndex;
        next        = anims[index].nextAnimation - model->animation;
        dir         = 1.0f;
        time        = (animFrame <= 0 ? -animFrame : (animFrame - anim->frameStart)) / 30.0f;
        timeMax     = (anim->frameEnd - anim->frameStart + lerpToNext) / 30.0f;
        framesCount = anim->frameEnd - anim->frameStart + 1;
        updateInfo();
        framePrev   = frameIndex - 1;
        getCommand(anim, frameIndex, &offset, &jump, NULL);
        return state = anim->state;
    }

    void playNext() {
        setAnim(next, anims[index].nextFrame);
    }

    void updateInfo() {
        ASSERT(model);
        ASSERT(anims);
        TR::Animation *anim = anims + index;

    //    framePrev  = frameIndex;
        frameIndex = int(time * 30.0f);

    // get count of real frames
        int fCount = (anim->frameEnd - anim->frameStart) / anim->frameRate + 1;
    // real frame index & lerp delta
        int fIndex = int(time * 30.0f) / anim->frameRate;
        int k = fIndex * anim->frameRate;
        delta = (time * 30.0f - k) / min((int)anim->frameRate, framesCount - k); // min is because in some cases framesCount > realFramesCount / frameRate * frameRate

    // size of frame (in bytes)
        int fSize = sizeof(TR::AnimFrame) + model->mCount * sizeof(uint16) * 2;

        int fIndexA = fIndex % fCount, fIndexB = (fIndex + 1) % fCount;
        frameA = (TR::AnimFrame*)&level->frameData[(anim->frameOffset + fIndexA * fSize) >> 1]; // >> 1 (div 2) because frameData is array of shorts
 
        int frameNext = frameIndex + 1;
        isPrepareToNext = !fIndexB;
        if (isPrepareToNext) {
            frameNext  = anim->nextFrame;
            anim       = &level->anims[anim->nextAnimation];
            frameNext -= anim->frameStart;
            fIndexB    = frameNext / anim->frameRate;            
        }

        getCommand(anim, frameNext, NULL, NULL, &flip);

        frameB = (TR::AnimFrame*)&level->frameData[(anim->frameOffset + fIndexB * fSize) >> 1];
    }

    bool isFrameActive(int index) {
        return index > framePrev && index <= frameIndex;
    }

    bool canSetState(int state) {
        TR::Animation *anim = anims + index;

        if (state == anim->state)
            return true;

        for (int i = 0; i < anim->scCount; i++) {
            TR::AnimState &s = level->states[anim->scOffset + i];
            if (s.state == state)
                for (int j = 0; j < s.rangesCount; j++) {
                    TR::AnimRange &range = level->ranges[s.rangesOffset + j];
                    if (anim->frameStart + frameIndex >= range.low && anim->frameStart + frameIndex <= range.high)
                        return true;
                }
        }

        return false;
    }

    bool setState(int state) {
        TR::Animation *anim = anims + index;

        if (state == anim->state)
            return true;

        bool exists = false;

        for (int i = 0; i < anim->scCount; i++) {
            TR::AnimState &s = level->states[anim->scOffset + i];
            if (s.state == state) {
                exists = true;
                for (int j = 0; j < s.rangesCount; j++) {
                    TR::AnimRange &range = level->ranges[s.rangesOffset + j];
                    if (anim->frameStart + frameIndex >= range.low && anim->frameStart + frameIndex <= range.high) {
                        setAnim(range.nextAnimation - model->animation, range.nextFrame);
                        break;
                    }
                }
            }
        }

        return exists;
    }

    float getSpeed() {
        TR::Animation *anim = anims + index;
        return anim->speed + anim->accel * (time * 30.0f);
    }

    void getCommand(TR::Animation *anim, int frameIndex, vec3 *offset, vec3 *jump, bool *flip) {
        int16 *ptr = &level->commands[anim->animCommand];

        if (offset) *offset = vec3(0.0f);
        if (flip)   *flip   = false;

        for (int i = 0; i < anim->acCount; i++) {
            int cmd = *ptr++; 
            switch (cmd) {
                case TR::ANIM_CMD_OFFSET :
                    if (offset) {
                        offset->x = (float)*ptr++;
                        offset->y = (float)*ptr++;
                        offset->z = (float)*ptr++;
                    } else
                        ptr += 3;
                    break;
                case TR::ANIM_CMD_JUMP :
                    if (jump) {
                        jump->y = (float)*ptr++;
                        jump->z = (float)*ptr++;
                    } else
                        ptr += 2;
                    break;                
                case TR::ANIM_CMD_SOUND  : ptr += 2; break;
                case TR::ANIM_CMD_EFFECT :
                    if (flip) {
                        int frame = (*ptr++) - anim->frameStart;
                        int fx    = (*ptr++) & 0x3FFF;
                        *flip     = fx == TR::EFFECT_ROTATE_180 && frame == frameIndex;
                    } else
                        ptr += 2;
                    break;
            }
        }
    }

    quat getJointRot(int joint) {
        return lerpAngle(frameA->getAngle(joint), frameB->getAngle(joint), delta);
    }

    mat4 getJoints(mat4 matrix, int joint, bool postRot = false, mat4 *joints = NULL) {
        TR::Animation *anim = anims + index;

        vec3 offset = isPrepareToNext ? this->offset : vec3(0.0f);
        matrix.translate(((vec3)frameA->pos).lerp(offset + frameB->pos, delta));

        TR::Node *node = (int)model->node < level->nodesDataSize ? (TR::Node*)&level->nodesData[model->node] : NULL;

        int sIndex = 0;
        mat4 stack[16];

        for (int i = 0; i < model->mCount; i++) {

            if (i > 0 && node) {
                TR::Node &t = node[i - 1];

                if (t.flags & 0x01) matrix = stack[--sIndex];
                if (t.flags & 0x02) stack[sIndex++] = matrix;

                ASSERT(sIndex >= 0 && sIndex < 16);

                matrix.translate(vec3((float)t.x, (float)t.y, (float)t.z));
            }

            if (i == joint && !postRot)
                return matrix;

            quat q;
            if (overrideMask & (1 << i))
                q = overrides[i];
            else
                q = getJointRot(i);
            matrix = matrix * mat4(q, vec3(0.0f));

            if (i == joint && postRot)
                return matrix;

            if (joints)
                joints[i] = matrix;
        }
        return matrix;
    }

    Box getBoundingBox(const vec3 &pos, int dir) {
        vec3 min = frameA->box.min().lerp(frameB->box.min(), delta);
        vec3 max = frameA->box.max().lerp(frameB->box.max(), delta);
        Box box(min, max);
        box.rotate90(dir);
        box.min += pos;
        box.max += pos;
        return box;
    }
};

#endif
#ifndef DEFMESH_H_INCLUDED
#define DEFMESH_H_INCLUDED

#include "../Pinocchio/attachment.h"
#include "DisplayMesh.h"
#include "motion.h"


class DeformableMesh : public DisplayMesh
{
public:
    DeformableMesh(const Mesh inMesh, const Skeleton &inOrigSkel, const vector<Vector3> &inMatch,
            const Attachment &inAttachment, Motion *inMotion = NULL) 
      : origSkel(inOrigSkel), match(inMatch), attachment(inAttachment), origMesh(inMesh), motion(inMotion) { }

    vector<Vector3> getSkel() const;
    const Skeleton &getOrigSkel() const { return origSkel; }
    const Attachment &getAttachment() const { return attachment; }
    const Mesh &getMesh() { updateMesh(); return curMesh; }

private:
    vector<Transform<> > computeTransforms() const;
    void updateMesh() const;

    Skeleton origSkel;
    vector<Vector3> match;
    Attachment attachment;
    Mesh origMesh;
    mutable Mesh curMesh;
    Motion *motion;
};

#endif //DEFMESH_H_INCLUDED

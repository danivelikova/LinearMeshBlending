#include "defmesh.h"
#include "../Pinocchio/intersector.h"

// Linear Blend Skinning
void DeformableMesh::updateMesh() const
{
    //get bones orientation from sample motion data
    vector<Transform<> > transforms = computeTransforms();
    curMesh = origMesh;
    for(int i=0; i< origMesh.vertices.size(); i++) {
        Vector3 updatedPos;
        Vector<double,-1> weights = attachment.getWeights(i);
        int l = weights.size();
        for(int j=0; j< weights.size(); j++) {
            updatedPos += weights[j] * (transforms[j] * origMesh.vertices[i].pos);
        }
        curMesh.vertices[i].pos = updatedPos;

    }
    curMesh.computeVertexNormals();
}

vector<Vector3> DeformableMesh::getSkel() const
{
    vector<Vector3> out = match;

    vector<Transform<> > t;
    t = computeTransforms();
        
    out[0] = t[0] * out[0];
    for(int i = 1; i < (int)out.size(); ++i) {
        // update bone positions by applying rotation and translations wrt orignal skeleton
        out[i] = t[ i - 1] * out[i];
    }
    
    return out;
}


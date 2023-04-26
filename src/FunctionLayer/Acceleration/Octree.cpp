#include "Octree.h"
#include <queue>
struct Octree::OctreeNode {
    AABB boundingBox;
    std::shared_ptr<OctreeNode> subNodes[8];
    int primCount = -1;
    int primIdxBuffer[ocLeafMaxSize];
};
static Point3f minP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

static Point3f maxP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}
Octree::OctreeNode * Octree::recursiveBuild(const AABB &aabb,
                       const std::vector<int> &primIdxBuffer) {
  //* todo 完成递归构建八叉树
  //* 构建方法请看实验手册
  //* 要注意的一种特殊是当节点的某个子包围盒和当前节点所有物体都相交，我们就不用细分了，当前节点作为叶子节点即可。
  auto n = primIdxBuffer.size();
  auto ret = new OctreeNode;
  ret->boundingBox = aabb;
  ret->primCount = n;
  if(n<ocLeafMaxSize){
    for(int i = 0;i<n;i++){
      ret->primIdxBuffer[i] = primIdxBuffer[i];
    }
    for(int i = 0;i<8;i++)
      ret->subNodes[i] = NULL;
  }
  else{
    AABB aabbSub[8];
    auto&& Pcenter = aabb.Center(); 
    auto&pMin = aabb.pMin, &pMax = aabb.pMax;
    aabbSub[0] = AABB(pMin,Pcenter);
    aabbSub[1] = AABB(minP(Pcenter,Point3f(pMax[0],pMin[1],pMin[2])),maxP(Pcenter,Point3f(pMax[0],pMin[1],pMin[2])));
    aabbSub[2] = AABB(minP(Pcenter,Point3f(pMin[0],pMax[1],pMin[2])),maxP(Pcenter,Point3f(pMin[0],pMax[1],pMin[2])));
    aabbSub[3] = AABB(minP(Pcenter,Point3f(pMax[0],pMax[1],pMin[2])),maxP(Pcenter,Point3f(pMax[0],pMax[1],pMin[2])));
    aabbSub[4] = AABB(minP(Pcenter,Point3f(pMin[0],pMin[1],pMax[2])),maxP(Pcenter,Point3f(pMin[0],pMin[1],pMax[2])));
    aabbSub[5] = AABB(minP(Pcenter,Point3f(pMax[0],pMin[1],pMax[2])),maxP(Pcenter,Point3f(pMax[0],pMin[1],pMax[2])));
    aabbSub[6] = AABB(minP(Pcenter,Point3f(pMin[0],pMax[1],pMax[2])),maxP(Pcenter,Point3f(pMin[0],pMax[1],pMax[2])));
    aabbSub[7] = AABB(Pcenter,pMax);
    std::vector<std::vector<int>> subBuffers(8);
    for(int i = 0;i<8;i++){
      for(auto& id:primIdxBuffer){
        if(shapes[id]->getAABB().Overlap(aabbSub[i])){
          subBuffers[i].push_back(id);
        }
      }
    }
    for(int i = 0;i<8;i++){
      ret->subNodes[i] = std::shared_ptr<OctreeNode>(recursiveBuild(aabbSub[i],subBuffers[i]));
    }
      
  }
  return ret;
}
void Octree::build() {
  //* 首先计算整个场景的范围
  for (const auto & shape : shapes) {
    //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
    //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
    //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
    //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
    shape->initInternalAcceleration();

    boundingBox.Expand(shape->getAABB());
  }

  //* 构建八叉树
  std::vector<int> primIdxBuffer(shapes.size());
  std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
  root = recursiveBuild(boundingBox, primIdxBuffer);
}

bool Octree::rayIntersect(Ray &ray, int *geomID, int *primID,
                          float *u, float *v) const {
  //*todo 完成八叉树求交
  std::function<bool(OctreeNode*)> DFS = [&](OctreeNode* t)->bool{
    bool ret = false;
    if(t->boundingBox.RayIntersect(ray,NULL,NULL)){
      if(t->subNodes[0]==NULL){
        for(int i = 0;i<t->primCount;i++){
          if(shapes[t->primIdxBuffer[i]]->rayIntersectShape(ray,primID,u,v)){
            *geomID = t->primIdxBuffer[i];
            ret = true;
          }
        }
      }else{
        for(int i = 0;i<8;i++){
          if(DFS(t->subNodes[i].get()))
            ret = true;
        }
      }
    }
    return ret;
  };
  return DFS(root);
}
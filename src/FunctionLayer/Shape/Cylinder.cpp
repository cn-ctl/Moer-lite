#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆柱范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    auto&& Tray = transform.inverseRay(ray);
    auto && A = Tray.direction[0]*Tray.direction[0]+Tray.direction[1]*Tray.direction[1],
        B = 2.0f*(Tray.origin[0]*Tray.direction[0]+Tray.origin[1]*Tray.direction[1]),
        C = Tray.origin[0]*Tray.origin[0] + Tray.origin[1]*Tray.origin[1] - radius*radius;
    float t0, t1;
    if(!Quadratic(A,B,C,&t0,&t1))
        return false;

    auto check = [&](float t)->bool{
        if(t<Tray.tNear||t>Tray.tFar)
            return false;

        auto&& point = Tray.at(t);
        if(point[2]<0||point[2]>height)
            return false;
        
        float phi = atan2(point[1],point[0]);
        if(phi < 0)
            phi += 2*PI;
        if(phi > phiMax)
            return false;

        ray.tFar = t;
        *u = phi/phiMax;
        *v = point[2]/height;
        *primID = 0;
        return true;
    };
    if(!check(t0))
        if(!check(t1))
            return false;
    return true;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------
    auto phi = u*phiMax;
    intersection->normal = transform.toWorld(Vector3f{(float)cos(phi),(float)sin(phi),0.f});

    intersection->position = transform.toWorld(Point3f{(float)cos(phi)*radius,(float)sin(phi)*radius,v*height});

    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")

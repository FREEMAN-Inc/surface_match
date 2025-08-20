#include <random>
#include <limits>
#include <filePLY.h>
#include <helper.h>
#include <ppf.h>
#include <util.h>
#include <Eigen/Geometry>
// Fit plane ax+by+cz+d=0 from 3 non-collinear points
inline Eigen::Vector4f fitPlane(const Eigen::Vector3f& p1,
                                const Eigen::Vector3f& p2,
                                const Eigen::Vector3f& p3) {
    Eigen::Vector3f n = (p2 - p1).cross(p3 - p1).normalized();
    float d = -n.dot(p1);
    return Eigen::Vector4f(n.x(), n.y(), n.z(), d);
}

// Compute distance of point to plane
inline float pointPlaneDist(const Eigen::Vector3f& p, const Eigen::Vector4f& plane) {
    return std::abs(plane.head<3>().dot(p) + plane[3]);
}

ppf::PointCloud removePlaneRANSAC(const ppf::PointCloud& cloud,
                             int maxIters = 2000,
                             float distThresh = 0.1f,  // 10 mm
                             Eigen::Vector3f axis = Eigen::Vector3f(0,0,1),
                             float epsAngleDeg = 10.0f,
                             int minInliers = 500)

{
    std::mt19937 rng(42);
    std::uniform_int_distribution<> uni(0, cloud.point.size()-1);

    int bestCount = 0;
    Eigen::Vector4f bestPlane;
    std::vector<int> bestInliers;

    for (int it=0; it<maxIters; ++it) {
        // pick 3 random points
        int i1 = uni(rng), i2 = uni(rng), i3 = uni(rng);
        if (i1==i2 || i1==i3 || i2==i3) continue;
        Eigen::Vector4f plane = fitPlane(cloud.point[i1], cloud.point[i2], cloud.point[i3]);

        // check orientation vs axis
        Eigen::Vector3f n = plane.head<3>().normalized();
        float angle = std::acos(std::abs(n.dot(axis))); // radians
        if (angle > epsAngleDeg * M_PI/180.f) continue;

        // count inliers
        std::vector<int> inliers;
        for (int j=0;j<(int)cloud.point.size();++j) {
            if (pointPlaneDist(cloud.point[j], plane) < distThresh) {
                inliers.push_back(j);
            }
        }
        if ((int)inliers.size() > bestCount) {
            bestCount = inliers.size();
            bestPlane = plane;
            bestInliers.swap(inliers);
        }
    }

    ppf::PointCloud out;
    out.viewPoint = cloud.viewPoint;
    if (bestCount < minInliers) {
        return cloud; // nothing removed
    }

    // Copy all points except inliers
    for (int i=0;i<(int)cloud.point.size();++i) {
        if (std::find(bestInliers.begin(), bestInliers.end(), i)==bestInliers.end()) {
            out.point.push_back(cloud.point[i]);
            if (cloud.hasNormal()) out.normal.push_back(cloud.normal[i]);
        }
    }
    return out;
}

ppf::PointCloud filterZAbove(const ppf::PointCloud& cloud, float zMin)
{
    ppf::PointCloud out;

    out.point.reserve(cloud.point.size());
    if (cloud.hasNormal())
        out.normal.reserve(cloud.normal.size());

    for (size_t i = 0; i < cloud.point.size(); ++i)
    {
        const Eigen::Vector3f& p = cloud.point[i];
        if (p.z() <= zMin)   // 過濾條件：z >= zMin
        {
            out.point.push_back(p);

            if (cloud.hasNormal() && i < cloud.normal.size())
                out.normal.push_back(cloud.normal[i]);
        }
    }

    // TODO: 如果 face / box 需要，也可在這裡更新
    return out;
}



int main(int argc, char *argv[]) {
    ppf::PointCloud scene;
    ppf::readPLY(argv[ 1 ], scene);

    ppf::PointCloud scenefilter;

    // 1. remove bottom horizontal plane
    scenefilter = removePlaneRANSAC(scene, 2000, 0.004f, Eigen::Vector3f(0,0,1), 12.0f, 1000);

    float zThreshold = 0.52f;  // 例如只保留  <= Zmax 的點
    ppf::PointCloud filtered = filterZAbove(scene, zThreshold);

    ppf::writePLY("scenefilter.ply", scenefilter);

    std::cout << "Original points: " << scene.point.size()
              << ", Filtered points: " << filtered.point.size() << std::endl;

}
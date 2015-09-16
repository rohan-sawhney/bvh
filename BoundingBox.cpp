#include "BoundingBox.h"
#include "Face.h"
#include <algorithm>
#define EPSILON 1e-6

BoundingBox::BoundingBox():
min(Eigen::Vector3d::Zero()),
max(Eigen::Vector3d::Zero()),
extent(Eigen::Vector3d::Zero())
{
    
}

BoundingBox::BoundingBox(const Eigen::Vector3d& min0, const Eigen::Vector3d& max0):
min(min0),
max(max0)
{
    extent = max - min;
}

BoundingBox::BoundingBox(const Eigen::Vector3d& p):
min(p),
max(p)
{
    extent = max - min;
}

void BoundingBox::expandToInclude(const Eigen::Vector3d& p)
{
    if (min.x() > p.x()) min.x() = p.x();
    if (min.y() > p.y()) min.y() = p.y();
    if (min.z() > p.z()) min.z() = p.z();
    
    if (max.x() < p.x()) max.x() = p.x();
    if (max.y() < p.y()) max.y() = p.y();
    if (max.z() < p.z()) max.z() = p.z();
    
    extent = max - min;
}

void BoundingBox::expandToInclude(const BoundingBox& b)
{
    if (min.x() > b.min.x()) min.x() = b.min.x();
    if (min.y() > b.min.y()) min.y() = b.min.y();
    if (min.z() > b.min.z()) min.z() = b.min.z();
    
    if (max.x() < b.max.x()) max.x() = b.max.x();
    if (max.y() < b.max.y()) max.y() = b.max.y();
    if (max.z() < b.max.z()) max.z() = b.max.z();
    
    extent = max - min;
}

int BoundingBox::maxDimension() const
{
    int result = 0;
    if (extent.y() > extent.x()) result = 1;
    if (extent.z() > extent.y() && extent.z() > extent.x()) result = 2;
    
    return result;
}

void BoundingBox::computeOrientedBox(std::vector<Eigen::Vector3d>& positions)
{
    // compute mean
    Eigen::Vector3d cm;
    cm.setZero();
    for (size_t i = 0; i < positions.size(); i++) {
        cm += positions[i];
    }
    cm /= (double)positions.size();
    
    // adjust for mean and compute covariance matrix
    Eigen::Matrix3d covariance;
    covariance.setZero();
    for (size_t i = 0; i < positions.size(); i++) {
        Eigen::Vector3d pAdg = positions[i] - cm;
        covariance += pAdg * pAdg.transpose();
    }
    covariance /= (double)positions.size();
    
    // compute eigenvectors for covariance matrix
    Eigen::EigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Matrix3d eigenVectors = solver.eigenvectors().real();
    
    // set axes
    eigenVectors.transpose();
    xAxis = eigenVectors.row(0);
    yAxis = eigenVectors.row(1);
    zAxis = eigenVectors.row(2);
    
    // project min and max points on each principal axis
    double min1 = INF, max1 = -INF;
    double min2 = INF, max2 = -INF;
    double min3 = INF, max3 = -INF;
    double d = 0.0;
    for (size_t i = 0; i < positions.size(); i++) {
        d = xAxis.dot(positions[i]);
        if (min1 > d) min1 = d;
        if (max1 < d) max1 = d;
        
        d = yAxis.dot(positions[i]);
        if (min2 > d) min2 = d;
        if (max2 < d) max2 = d;
        
        d = zAxis.dot(positions[i]);
        if (min3 > d) min3 = d;
        if (max3 < d) max3 = d;
    }
    
    // set center and halflengths
    center = (xAxis*(min1 + max1) + yAxis*(min2 + max2) + zAxis*(min3 + max3)) /2;
    halfLx = (max1 - min1)/2; halfLy = (max2 - min2)/2; halfLz = (max3 - min3)/2;
}

bool comparePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    return p1.x() < p2.x() || (p1.x() == p2.x() && p1.z() < p2.z());
}

double cross2d(const Eigen::Vector3d& o, const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
    return (p.x() - o.x()) * (q.z() - o.z()) - (p.z() - o.z()) * (q.x() - o.x());
}

std::vector<Eigen::Vector3d> generateConvexHull(std::vector<Eigen::Vector3d>& positions)
{
    // Andrew's Monotone Chain Algorithm
    std::sort(positions.begin(), positions.end(), comparePoints);
    
    int n = (int)positions.size();
    std::vector<Eigen::Vector3d> hullPoints(2*n);
    int k = 0;
    
    // build lower half
    for (int i = 0; i < n; i++) {
        // while there are at least 2 points in the array and the sequence of those points and
        // point[i] does not make a anti-clockwise turn
        while (k >= 2 &&
               cross2d(hullPoints[k-2], hullPoints[k-1], positions[i]) <= 0) {
            k--;
        }
        
        hullPoints[k++] = positions[i];
    }
    
    // build upper half
    for (int i = n-2, t = k+1; i >= 0; i--) {
        // while there are at least 2 points in the array and the sequence of those points and
        // point[i] does not make a anti-clockwise turn
        while (k >= t &&
               cross2d(hullPoints[k-2], hullPoints[k-1], positions[i]) <= 0) {
            k--;
        }
        
        hullPoints[k++] = positions[i];
    }
    
    hullPoints.resize(k);
    return hullPoints;
}

void BoundingBox::computeXZOrientedBox(std::vector<Eigen::Vector3d>& positions)
{
    orientedPoints = generateConvexHull(positions);
}

bool BoundingBox::intersect(const BoundingBox& boundingBox, double& dist) const
{
    Eigen::Vector3d bMin = boundingBox.min;
    Eigen::Vector3d bMax = boundingBox.max;
    
    if (((min.x() <= bMin.x() && bMin.x() <= max.x()) || (bMin.x() <= min.x() && min.x() <= bMax.x())) &&
        ((min.y() <= bMin.y() && bMin.y() <= max.y()) || (bMin.y() <= min.y() && min.y() <= bMax.y())) &&
        ((min.z() <= bMin.z() && bMin.z() <= max.z()) || (bMin.z() <= min.z() && min.z() <= bMax.z()))) {
        
        Eigen::Vector3d v = ((min + max)/2) - ((bMin + bMax)/2);
        dist = v.norm();
        return true;
    }
    
    return false;
}

bool BoundingBox::intersectOriented(const BoundingBox& boundingBox, double& dist) const
{
    // Uses Separating Axis Theorem to detect intersection
    Eigen::Vector3d diff = boundingBox.center - center;
    
    // Ax
    double dotAx = diff.dot(xAxis);
    double rxx = xAxis.dot(boundingBox.xAxis);
    double rxy = xAxis.dot(boundingBox.yAxis);
    double rxz = xAxis.dot(boundingBox.zAxis);
    
    if (fabs(dotAx) > (halfLx + fabs(boundingBox.halfLx * rxx) +
                       fabs(boundingBox.halfLy * rxy) +
                       fabs(boundingBox.halfLz * rxz))) return false;
    
    // Ay
    double dotAy = diff.dot(yAxis);
    double ryx = yAxis.dot(boundingBox.xAxis);
    double ryy = yAxis.dot(boundingBox.yAxis);
    double ryz = yAxis.dot(boundingBox.zAxis);
    
    if (fabs(dotAy) > (halfLy + fabs(boundingBox.halfLx * ryx) +
                       fabs(boundingBox.halfLy * ryy) +
                       fabs(boundingBox.halfLz * ryz))) return false;
    
    // Az
    double dotAz = diff.dot(zAxis);
    double rzx = zAxis.dot(boundingBox.xAxis);
    double rzy = zAxis.dot(boundingBox.yAxis);
    double rzz = zAxis.dot(boundingBox.zAxis);
    
    if (fabs(dotAz) > (halfLz + fabs(boundingBox.halfLx * rzx) +
                       fabs(boundingBox.halfLy * rzy) +
                       fabs(boundingBox.halfLz * rzz))) return false;
    
    // Bx
    if (fabs(diff.dot(boundingBox.xAxis)) > (boundingBox.halfLx + fabs(halfLx * rxx) +
                                             fabs(halfLy * ryx) +
                                             fabs(halfLz * rzx))) return false;
    
    // By
    if (fabs(diff.dot(boundingBox.yAxis)) > (boundingBox.halfLy + fabs(halfLx * rxy) +
                                             fabs(halfLy * ryy) +
                                             fabs(halfLz * rzy))) return false;
    
    // Bz
    if (fabs(diff.dot(boundingBox.zAxis)) > (boundingBox.halfLz + fabs(halfLx * rxz) +
                                             fabs(halfLy * ryz) +
                                             fabs(halfLz * rzz))) return false;
    
    // Ax x Bx
    if (fabs(dotAz*ryx - dotAy*rzx) > fabs(halfLy * rzx) +
        fabs(halfLz * ryx) +
        fabs(boundingBox.halfLy * rxz) +
        fabs(boundingBox.halfLz * rxy)) return false;
    
    // Ax x By
    if (fabs(dotAz*ryy - dotAy*rzy) > fabs(halfLy * rzy) +
        fabs(halfLz * ryy) +
        fabs(boundingBox.halfLx * rxz) +
        fabs(boundingBox.halfLz * rxx)) return false;
    
    // Ax x Bz
    if (fabs(dotAz*ryz - dotAy*rzz) > fabs(halfLy * rzz) +
        fabs(halfLz * ryz) +
        fabs(boundingBox.halfLx * rxy) +
        fabs(boundingBox.halfLy * rxx)) return false;
    
    // Ay x Bx
    if (fabs(dotAx*rzx - dotAz*rxx) > fabs(halfLx * rzx) +
        fabs(halfLz * rxx) +
        fabs(boundingBox.halfLy * ryz) +
        fabs(boundingBox.halfLz * ryy)) return false;
    
    // Ay x By
    if (fabs(dotAx*rzy - dotAz*rxy) > fabs(halfLx * rzy) +
        fabs(halfLz * rxy) +
        fabs(boundingBox.halfLx * ryz) +
        fabs(boundingBox.halfLz * ryx)) return false;
    
    // Ay x Bz
    if (fabs(dotAx*rzz - dotAz*rxz) > fabs(halfLx * rzz) +
        fabs(halfLz * rxz) +
        fabs(boundingBox.halfLx * ryy) +
        fabs(boundingBox.halfLy * ryx)) return false;
    
    // Az x Bx
    if (fabs(dotAy*rxx - dotAx*ryx) > fabs(halfLx * ryx) +
        fabs(halfLy * rxx) +
        fabs(boundingBox.halfLy * rzz) +
        fabs(boundingBox.halfLz * rzy)) return false;
    
    // Az x By
    if (fabs(dotAy*rxy - dotAx*ryy) > fabs(halfLx * ryy) +
        fabs(halfLy * rxy) +
        fabs(boundingBox.halfLx * rzz) +
        fabs(boundingBox.halfLz * rzx)) return false;
    
    // Az x Bz
    if (fabs(dotAy*rxz - dotAx*ryz) > fabs(halfLx * ryz) +
        fabs(halfLy * rxz) +
        fabs(boundingBox.halfLx * rzy) +
        fabs(boundingBox.halfLy * rzx)) return false;
    
    dist = diff.norm();
    
    // intersection
    return true;
}

bool BoundingBox::intersect(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, double& dist) const
{
    double ox = origin.x();
    double dx = direction.x();
    double tMin, tMax;
    if (dx >= 0) {
        tMin = (min.x() - ox) / dx;
        tMax = (max.x() - ox) / dx;
        
    } else {
        tMin = (max.x() - ox) / dx;
        tMax = (min.x() - ox) / dx;
    }
    
    double oy = origin.y();
    double dy = direction.y();
    double tyMin, tyMax;
    if (dy >= 0) {
        tyMin = (min.y() - oy) / dy;
        tyMax = (max.y() - oy) / dy;
        
    } else {
        tyMin = (max.y() - oy) / dy;
        tyMax = (min.y() - oy) / dy;
    }
    
    if (tMin > tyMax || tyMin > tMax) {
        return false;
    }
    
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;
    
    double oz = origin.z();
    double dz = direction.z();
    double tzMin, tzMax;
    if (dz >= 0) {
        tzMin = (min.z() - oz) / dz;
        tzMax = (max.z() - oz) / dz;
        
    } else {
        tzMin = (max.z() - oz) / dz;
        tzMax = (min.z() - oz) / dz;
    }
    
    if (tMin > tzMax || tzMin > tMax) {
        return false;
    }
    
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    
    dist = tMin;
    return true;
}

bool overlap(const Eigen::Vector3d& axis, const Eigen::Vector3d& diff, const Eigen::Vector3d& direction,
             const double halfL, double& minT, double& maxS)
{
    double e = axis.dot(diff);
    double f = axis.dot(direction);
    
    // ray is parallel
    if (f > -EPSILON && f < EPSILON) {
        // ray passes by box
        if (-e - halfL > 0.0 || -e + halfL > 0.0) {
            return false;
        }
        return true;
    }
    
    double s = (e - halfL)/f;
    double t = (e + halfL)/f;
    
    // fix order
    if (s > t) std::swap(s, t);
    
    // adjust min and max values
    if (s > maxS) maxS = s;
    if (t < minT) minT = t;
    
    // intersection failure
    if (minT < 0.0 || maxS > minT) return false;
    
    // overlap
    return true;
}

bool BoundingBox::intersectOriented(const Eigen::Vector3d& origin,
                                    const Eigen::Vector3d& direction, double& dist) const
{
    Eigen::Vector3d diff = center - origin;
    double minT = INF, maxS = -INF;
    
    if (overlap(xAxis, diff, direction, halfLx, minT, maxS) &&
        overlap(yAxis, diff, direction, halfLy, minT, maxS) &&
        overlap(zAxis, diff, direction, halfLz, minT, maxS)) return true;
    
    // no intersection
    return false;
    
}

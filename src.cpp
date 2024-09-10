#include <iostream>
#include <math.h>
#include <vector>
#include <string.h>
#include <algorithm>
#include "head.h"

Point64D addPoints(Point64D p1, Point64D p2) {
    Point64D result;
    result.x = p1.x + p2.x;
    result.y = p1.y + p2.y;
    return result;
}

Point64D subtractPoints(Point64D p1, Point64D p2) {
    Point64D result;
    result.x = p1.x - p2.x;
    result.y = p1.y - p2.y;
    return result;
}

double dot_2d(const Point64D *A, const Point64D *B) {
    return A->x * B->x + A->y * B->y;
}


double cross_2d( Point64D *A, Point64D *B) {
   return A->x * B->y - A->y * B->x;
}

void get_rotated_vertices(RotatedBox32F *box, Point64D pts[4]) {
    double theta = box->a;
    double cosTheta2 = (double)cos(theta) * 0.5f;
    double sinTheta2 = (double)sin(theta) * 0.5f;

    pts[0].x = box->x_ctr + sinTheta2 * box->h + cosTheta2 * box->w;
    pts[0].y = box->y_ctr + cosTheta2 * box->h - sinTheta2 * box->w;
    pts[1].x = box->x_ctr - sinTheta2 * box->h + cosTheta2 * box->w;
    pts[1].y = box->y_ctr - cosTheta2 * box->h - sinTheta2 * box->w;
    pts[2].x = 2 * box->x_ctr - pts[0].x;
    pts[2].y = 2 * box->y_ctr - pts[0].y;
    pts[3].x = 2 * box->x_ctr - pts[1].x;
    pts[3].y = 2 * box->y_ctr - pts[1].y;
}

int get_intersection_points(const Point64D pts1[4], const Point64D pts2[4], Point64D intersections[24]) {
    Point64D vec1[4], vec2[4];
    for (int i = 0; i < 4; i++) {
        vec1[i] = subtractPoints(pts1[(i + 1) % 4], pts1[i]);
        vec2[i] = subtractPoints(pts2[(i + 1) % 4], pts2[i]);
    }

    int num = 0; 
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double det = cross_2d(&vec2[j], &vec1[i]);

            if (fabs(det) <= 1e-14) {
                continue;
            }

            Point64D vec12 = subtractPoints(pts2[j], pts1[i]);

            double t1 = cross_2d(&vec2[j], &vec12) / det;
            double t2 = cross_2d(&vec1[i], &vec12) / det;

            if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f) {
                intersections[num].x = pts1[i].x + vec1[i].x * t1;
                intersections[num].y = pts1[i].y + vec1[i].y * t1;
                num++;
            }
        }
    }

    {
        Point64D AB = vec2[0];
        Point64D DA = vec2[3];
        double ABdotAB = dot_2d(&AB, &AB);
        double ADdotAD = dot_2d(&DA, &DA);
        for (int i = 0; i < 4; i++) {
            Point64D AP = subtractPoints(pts1[i], pts2[0]);

            double APdotAB = dot_2d(&AP, &AB);
            double APdotAD = -dot_2d(&AP, &DA);

            if ((APdotAB >= 0) && (APdotAD >= 0) && (APdotAB <= ABdotAB) &&
                (APdotAD <= ADdotAD)) {
                intersections[num] = pts1[i];
                num++;
            }
        }
    }

    {
        Point64D AB = vec1[0];
        Point64D DA = vec1[3];
        double ABdotAB = dot_2d(&AB, &AB);
        double ADdotAD = dot_2d(&DA, &DA);
        for (int i = 0; i < 4; i++) {
            Point64D AP = subtractPoints(pts2[i], pts1[0]);

            double APdotAB = dot_2d(&AP, &AB);
            double APdotAD = -dot_2d(&AP, &DA);

            if ((APdotAB >= 0) && (APdotAD >= 0) && (APdotAB <= ABdotAB) &&
                (APdotAD <= ADdotAD)) {
                intersections[num] = pts2[i];
                num++;
            }
        }
    }

    return num;
}


int point_order_compare(const void *a, const void *b) {
    Point64D A = *((Point64D *)a);
    Point64D B = *((Point64D *)b);

    double temp = cross_2d(&A, &B);
    if (fabs(temp) < 1e-6) {
        return dot_2d(&A, &A) - dot_2d(&B, &B);
    } else {
        return temp < 0 ? 1 : -1;
    }
}


int convex_hull_graham(const Point64D p[24], const int num_in, Point64D q[24], int shift_to_zero) {

    int t = 0;
    for (int i = 1; i < num_in; i++) {
        if (p[i].y < p[t].y || (p[i].y == p[t].y && p[i].x < p[t].x)) {
            t = i;
        }
    }
    Point64D start = p[t];

    for (int i = 0; i < num_in; i++) {
        q[i] = subtractPoints(p[i], start);
    }

    Point64D tmp = q[0];
    q[0] = q[t];
    q[t] = tmp;

    double dist[24];

    qsort(q + 1, num_in - 1, sizeof(Point64D), point_order_compare);

    for (int i = 0; i < num_in; i++) {
        dist[i] = dot_2d(&q[i], &q[i]);
    }

    int k;
    for (k = 1; k < num_in; k++) {
        if (dist[k] > 1e-8) {
            break;
        }
    }

    if (k == num_in) {
        q[0] = p[t];
        return 1;
    }

    q[1] = q[k];
    int m = 2;

    for (int i = k + 1; i < num_in; i++) {
        while (m > 1) {
            Point64D q1 = subtractPoints(q[i], q[m - 2]);
            Point64D q2 = subtractPoints(q[m - 1], q[m - 2]);
            if (q1.x * q2.y >= q2.x * q1.y)
                m--;
            else
                break;
        }

        q[m++] = q[i];
    }

    if (!shift_to_zero) {
        for (int i = 0; i < m; i++) {
            q[i] = addPoints(q[i], start);
        }
    }

    return m;
}



double polygon_area(const Point64D q[24], const int m) {
    if (m <= 2) {
        return 0;
    }

    double area = 0.0;
    for (int i = 1; i < m - 1; i++) {
        Point64D diff1 = subtractPoints(q[i], q[0]);
        Point64D diff2 = subtractPoints(q[i + 1], q[0]);
        area += fabs(cross_2d(&diff1, &diff2));
    }

    return area / 2.0;
}


double rotated_boxes_intersection(Point64D *pts1, Point64D *pts2) {
    Point64D intersectPts[24], orderedPts[24];

    int num = get_intersection_points(pts1, pts2, intersectPts);

    if (num <= 2) {
        return 0.0;
    }

    int num_convex = convex_hull_graham(intersectPts, num, orderedPts, 1);

    auto res = polygon_area(orderedPts, num_convex);

    return res;
}


double single_box_iou_rotated(RotatedBox32F box1_raw, RotatedBox32F box2_raw, Point64D *pts1, Point64D *pts2) {

    double area1 = box1_raw.w * box1_raw.h;
    double area2 = box2_raw.w * box2_raw.h;

    if (area1 < 1e-14 || area2 < 1e-14) {
        return 0.f;
    }

    double intersection = rotated_boxes_intersection(pts1, pts2);
    double iou = intersection / (area1 + area2 - intersection);
    return iou;
}

std::vector<int> nms_rotated(std::vector<std::vector<float>>& boxes, std::vector<float>& scores, float threshold) {
    std::vector<int> keep_index;
    if (boxes.size() != scores.size()){
      throw std::invalid_argument("The lengths of boxes and scores are inconsistent");
    }
    int dets_num = scores.size(); 
    int dets_dim = 5;
    int keep[dets_num];
    int order[dets_num];

    for (int i = 0; i < dets_num; ++i)
    {
        order[i] = i;
    }

    double suppressed[dets_num];
    memset(suppressed, 0, sizeof(float)*dets_num);

    std::sort(order, order+dets_num, [&scores](int i, int j){
      return scores[i]>scores[j];
    });

    Point64D pts_all[dets_num][4];
    RotatedBox32F rot_boxes[dets_num];

    for (int i = 0; i < dets_num; ++i){
      rot_boxes[i].x_ctr = boxes[i][0];
      rot_boxes[i].y_ctr = boxes[i][1];
      rot_boxes[i].w = boxes[i][2];
      rot_boxes[i].h = boxes[i][3];
      rot_boxes[i].a = boxes[i][4];
      get_rotated_vertices(&rot_boxes[i], pts_all[i]);
    }

    int k = 0;
    for (int i = 0; i < dets_num; ++i) {
        int idx = order[i];
        if (suppressed[idx] == 1) {
            continue;
        }

        keep[k] = idx;
        k += 1;

        for (int j = i + 1; j < dets_num; ++j) {
            int next_idx = order[j];

            if (suppressed[next_idx] == 1) {
                continue;
            }

            double iou = single_box_iou_rotated(rot_boxes[idx], rot_boxes[next_idx], pts_all[idx], pts_all[next_idx]);
 
            if (iou >= threshold) {
                suppressed[next_idx] = 1;
            }
        }
    }

    for (int i = 0; i < k; i++) {
       keep_index.emplace_back(keep[i]);
    }

    return keep_index;
}

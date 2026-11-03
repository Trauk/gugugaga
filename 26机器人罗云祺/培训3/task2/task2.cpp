#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int   kHoldFrames   = 6;
constexpr float kSmoothCenter = 0.35f;
constexpr float kSmoothSize   = 0.28f;
constexpr float kSmoothAngle  = 0.25f;

struct LightBar 
{
    cv::RotatedRect rect;
    cv::Point2f center;
    double area = 0.0;
    float length = 0.0f;
    float width = 0.0f;
    float angle = 0.0f;
};

struct BarGeom 
{
    cv::Point2f center;
    float length = 0.0f;
    float width = 0.0f;
    float angle = 0.0f;
};

struct ArmorObservation 
{
    bool valid = false;
    BarGeom left;
    BarGeom right;
};

struct ArmorState 
{
    bool valid = false;
    BarGeom left;
    BarGeom right;
    int lostFrames = 0;
};

static float normalizeAngle180(float a)
{
    a = std::fmod(a, 180.0f);
    if (a < 0.0f) a += 180.0f;
    return a;
}

static float angleDelta180(float from, float to)
{
    float d = std::fmod(to - from + 90.0f, 180.0f);
    if (d < 0.0f) d += 180.0f;
    return d - 90.0f;
}

static float acuteAngleDiff(float a, float b)
{
    float d = std::fabs(a - b);
    d = std::fmod(d, 180.0f);
    return std::min(d, 180.0f - d);
}

static BarGeom barFromRect(const cv::RotatedRect& r)
{
    BarGeom g;
    g.center = r.center;
    g.length = std::max(r.size.width, r.size.height);
    g.width = std::min(r.size.width, r.size.height);
    g.angle = normalizeAngle180(
        (r.size.width >= r.size.height) ? r.angle : r.angle + 90.0f);
    return g;
}

static cv::RotatedRect rectFromBar(const BarGeom& g)
{
    // width 方向作为长轴，height 方向作为短轴
    return cv::RotatedRect(g.center, cv::Size2f(g.length, g.width), g.angle);
}

static std::vector<LightBar> findRedBars(const cv::Mat& frame)
{
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red1, red2, mask;
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(12, 255, 255), red1);
    cv::inRange(hsv, cv::Scalar(168, 70, 50), cv::Scalar(180, 255, 255), red2);
    cv::bitwise_or(red1, red2, mask);

    const cv::Mat k3 =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    const cv::Mat k7 =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k3);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k7);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    const double minArea =
        std::max(70.0, static_cast<double>(frame.total()) * 3e-5);

    std::vector<LightBar> bars;

    for (const auto& c : contours) {
        std::vector<cv::Point> hull;
        cv::convexHull(c, hull);
        if (hull.size() < 4) continue;

        const double area = cv::contourArea(hull);
        if (area < minArea) continue;

        const cv::RotatedRect rr = cv::minAreaRect(hull);

        const float w = rr.size.width;
        const float h = rr.size.height;
        if (w < 1.0f || h < 1.0f) continue;

        const float longSide = std::max(w, h);
        const float shortSide = std::min(w, h);

        if (shortSide < 5.0f) continue;
        if (longSide / shortSide < 1.8f) continue;

        if (area / (static_cast<double>(longSide) * shortSide) < 0.35) {
            continue;
        }

        LightBar b;
        b.rect = rr;
        b.center = rr.center;
        b.area = area;
        b.length = longSide;
        b.width = shortSide;
        b.angle = normalizeAngle180(
            (rr.size.width >= rr.size.height) ? rr.angle
                                              : rr.angle + 90.0f);
        bars.push_back(b);
    }

    std::sort(bars.begin(), bars.end(),
              [](const LightBar& a, const LightBar& b) {
                  return a.area > b.area;
              });

    if (bars.size() > 16) bars.resize(16);
    return bars;
}

static bool selectArmorPair(const std::vector<LightBar>& bars,
                            bool hasPrev,
                            const cv::Point2f& prevLeft,
                            const cv::Point2f& prevRight,
                            LightBar& left,
                            LightBar& right)
{
    double bestScore = -1e100;
    int bestI = -1;
    int bestJ = -1;

    for (size_t i = 0; i < bars.size(); ++i) {
        for (size_t j = i + 1; j < bars.size(); ++j) {
            const LightBar& A = bars[i];
            const LightBar& B = bars[j];

            const float maxH = std::max(A.length, B.length);
            const float minH = std::min(A.length, B.length);
            if (minH < 1.0f) continue;

            // 1. 长度差比例过滤
            const float hRatio = (maxH - minH) / minH;
            if (hRatio > 0.55f) continue;

            // 2. 中心 Y 差过滤
            const float dx = std::fabs(A.center.x - B.center.x);
            const float dy = std::fabs(A.center.y - B.center.y);
            const float avgH = 0.5f * (A.length + B.length);
            const float yRatio = dy / avgH;
            if (yRatio > 0.35f) continue;

            // 3. 距离与灯条高度比例过滤
            const float dist = std::sqrt(dx * dx + dy * dy);
            const float distRatio = dist / avgH;
            if (distRatio < 0.8f || distRatio > 4.5f) continue;

            // 4. 平行度过滤
            const float ad = acuteAngleDiff(A.angle, B.angle);
            if (ad > 25.0f) continue;

            // 5. 宽度差过滤
            const float maxW = std::max(A.width, B.width);
            const float minW = std::min(A.width, B.width);
            if (minW < 1.0f || maxW / minW > 3.0f) continue;

            // 6. 面积差过滤
            if (std::max(A.area, B.area) / std::min(A.area, B.area) > 6.0) {
                continue;
            }

            double score = std::min(A.area, B.area);

            score *= 1.0 - 0.35 * static_cast<double>(hRatio);
            score *= 1.0 - 0.35 * static_cast<double>(yRatio);
            score *= 1.0 - 0.25 * static_cast<double>(ad) / 25.0;
            score *= 1.0 -
                     0.15 * std::fabs(static_cast<double>(distRatio) - 2.0) /
                         2.5;

            // 7. 优先保持上一帧的左右灯条，避免配对跳变
            if (hasPrev) {
                const LightBar* L = &A;
                const LightBar* R = &B;
                if (L->center.x > R->center.x) std::swap(L, R);

                const double prior =
                    cv::norm(L->center - prevLeft) +
                    cv::norm(R->center - prevRight);

                score /= (1.0 + 0.8 * prior / static_cast<double>(avgH));
            }

            if (score > bestScore) {
                bestScore = score;
                bestI = static_cast<int>(i);
                bestJ = static_cast<int>(j);
            }
        }
    }

    if (bestI < 0) return false;

    if (bars[bestI].center.x < bars[bestJ].center.x) 
    {
        left = bars[bestI];
        right = bars[bestJ];
    } else {
        left = bars[bestJ];
        right = bars[bestI];
    }
    return true;
}

static void getOuterPoints(const cv::RotatedRect& self,
                           const cv::RotatedRect& other,
                           cv::Point2f& top,
                           cv::Point2f& bottom)
{
    cv::Point2f p[4];
    self.points(p);

    struct Edge {
        int i0;
        int i1;
        float len;
    };

    std::vector<Edge> edges;
    edges.reserve(4);

    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        Edge e;
        e.i0 = i;
        e.i1 = j;
        e.len = static_cast<float>(cv::norm(p[j] - p[i]));
        edges.push_back(e);
    }

    std::sort(edges.begin(), edges.end(),
              [](const Edge& a, const Edge& b) {
                  return a.len > b.len;
              });

    const Edge& e0 = edges[0];
    const Edge& e1 = edges[1];

    const auto distSum = [&](const Edge& e) {
        return cv::norm(p[e.i0] - other.center) +
               cv::norm(p[e.i1] - other.center);
    };

    const Edge& outer = (distSum(e0) >= distSum(e1)) ? e0 : e1;

    cv::Point2f a = p[outer.i0];
    cv::Point2f b = p[outer.i1];

    if (a.y <= b.y) {
        top = a;
        bottom = b;
    } else {
        top = b;
        bottom = a;
    }
}

static ArmorObservation detectArmor(const cv::Mat& frame,
                                    const ArmorState& state)
{
    ArmorObservation obs;

    const std::vector<LightBar> bars = findRedBars(frame);

    LightBar left;
    LightBar right;

    const bool hasPrev = state.valid;
    const cv::Point2f prevLeft =
        hasPrev ? state.left.center : cv::Point2f(0.0f, 0.0f);
    const cv::Point2f prevRight =
        hasPrev ? state.right.center : cv::Point2f(0.0f, 0.0f);

    if (selectArmorPair(bars, hasPrev, prevLeft, prevRight, left, right)) {
        obs.valid = true;
        obs.left = barFromRect(left.rect);
        obs.right = barFromRect(right.rect);
    }

    return obs;
}

static BarGeom smoothBar(const BarGeom& oldG, const BarGeom& newG)
{
    if (oldG.length < 1.0f) return newG;

    const float move =
        static_cast<float>(cv::norm(newG.center - oldG.center));

    // 发生大跳变时直接重置，避免慢慢拖过去
    if (move > 1.2f * oldG.length ||
        newG.length > 1.8f * oldG.length ||
        newG.length < 0.55f * oldG.length) {
        return newG;
    }

    BarGeom g;

    const float alphaPos =
        (move > 0.25f * oldG.length) ? 0.70f : kSmoothCenter;

    g.center.x =
        oldG.center.x + alphaPos * (newG.center.x - oldG.center.x);
    g.center.y =
        oldG.center.y + alphaPos * (newG.center.y - oldG.center.y);

    g.length = oldG.length + kSmoothSize * (newG.length - oldG.length);
    g.width = oldG.width + kSmoothSize * (newG.width - oldG.width);
    g.angle = normalizeAngle180(
        oldG.angle + kSmoothAngle * angleDelta180(oldG.angle, newG.angle));

    return g;
}

static void updateArmorState(ArmorState& state,
                             const ArmorObservation& obs)
{
    if (!obs.valid) {
        if (state.valid) {
            ++state.lostFrames;
            if (state.lostFrames > kHoldFrames) {
                state.valid = false;
                state.lostFrames = 0;
            }
        }
        return;
    }

    if (!state.valid) {
        state.left = obs.left;
        state.right = obs.right;
        state.valid = true;
        state.lostFrames = 0;
        return;
    }

    state.left = smoothBar(state.left, obs.left);
    state.right = smoothBar(state.right, obs.right);
    state.lostFrames = 0;
}

static void drawYellowBox(cv::Mat& img, const cv::RotatedRect& r)
{
    cv::Point2f pts[4];
    r.points(pts);

    for (int i = 0; i < 4; ++i) {
        cv::line(img,
                 pts[i],
                 pts[(i + 1) % 4],
                 cv::Scalar(0, 255, 255),
                 2,
                 cv::LINE_AA);
    }
}

static void drawLabel(cv::Mat& img,
                      const std::string& text,
                      const cv::Point2f& p,
                      const cv::Point& offset)
{
    const double fontScale = 0.7;
    const int thickness = 2;

    int baseline = 0;
    const cv::Size textSize =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                        fontScale, thickness, &baseline);

    int x = cvRound(p.x) + offset.x;
    int y = cvRound(p.y) + offset.y;

    x = std::max(2, std::min(x, img.cols - textSize.width - 2));
    y = std::max(textSize.height + 2,
                 std::min(y, img.rows - baseline - 2));

    cv::putText(img,
                text,
                cv::Point(x, y),
                cv::FONT_HERSHEY_SIMPLEX,
                fontScale,
                cv::Scalar(0, 255, 0),
                thickness,
                cv::LINE_AA);
}

static void drawArmor(cv::Mat& frame, const ArmorState& state)
{
    if (!state.valid) {
        cv::putText(frame,
                    "Armor not found",
                    cv::Point(30, 50),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 255),
                    2,
                    cv::LINE_AA);
        return;
    }

    const cv::RotatedRect leftRect = rectFromBar(state.left);
    const cv::RotatedRect rightRect = rectFromBar(state.right);

    drawYellowBox(frame, leftRect);
    drawYellowBox(frame, rightRect);

    cv::Point2f p1, p2, p3, p4;

    getOuterPoints(leftRect, rightRect, p1, p4);

    getOuterPoints(rightRect, leftRect, p2, p3);

    const cv::Scalar green(0, 255, 0);

    cv::line(frame, p1, p2, green, 2, cv::LINE_AA);
    cv::line(frame, p2, p3, green, 2, cv::LINE_AA);
    cv::line(frame, p3, p4, green, 2, cv::LINE_AA);
    cv::line(frame, p4, p1, green, 2, cv::LINE_AA);

    cv::circle(frame, p1, 6, green, cv::FILLED, cv::LINE_AA);
    cv::circle(frame, p2, 6, green, cv::FILLED, cv::LINE_AA);
    cv::circle(frame, p3, 6, green, cv::FILLED, cv::LINE_AA);
    cv::circle(frame, p4, 6, green, cv::FILLED, cv::LINE_AA);

    drawLabel(frame, "P1", p1, cv::Point(-48, -12));
    drawLabel(frame, "P4", p4, cv::Point(-48, 26));
    drawLabel(frame, "P2", p2, cv::Point(12, -12));
    drawLabel(frame, "P3", p3, cv::Point(12, 26));
}

}

int main(int argc, char** argv)
{
    const std::string input =
        (argc > 1) ? argv[1] : "test_video.mp4";
    const std::string output =
        (argc > 2) ? argv[2] : "armor_result.mp4";

    cv::VideoCapture cap(input);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open video: " << input << std::endl;
        return -1;
    }

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "Cannot read first frame." << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (!(fps > 1.0) || fps > 240.0) fps = 30.0;

    cv::VideoWriter writer;
    std::string actualOutput = output;

    writer.open(output,
                cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps,
                frame.size());

    if (!writer.isOpened()) {
        actualOutput = "armor_result.avi";
        writer.open(actualOutput,
                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    fps,
                    frame.size());
    }

    if (!writer.isOpened()) {
        std::cerr << "Warning: cannot open output video, display only."
                  << std::endl;
    }

    ArmorState state;
    const double frameIntervalMs = 1000.0 / fps;

    while (true) {
        const auto begin = std::chrono::steady_clock::now();

        const ArmorObservation obs = detectArmor(frame, state);
        updateArmorState(state, obs);
        drawArmor(frame, state);

        if (writer.isOpened()) {
            writer.write(frame);
        }

        cv::imshow("Armor Detection", frame);

        const auto end = std::chrono::steady_clock::now();
        const double costMs =
            std::chrono::duration<double, std::milli>(end - begin).count();

        const int waitMs = std::max(
            1,
            static_cast<int>(
                std::lround(frameIntervalMs - costMs)));

        const int key = cv::waitKey(waitMs);
        if (key == 27 || key == 'q') break;

        if (!cap.read(frame) || frame.empty()) break;
    }

    cap.release();
    writer.release();
    cv::destroyAllWindows();

    std::cout << "Output: " << actualOutput << std::endl;
    return 0;
}

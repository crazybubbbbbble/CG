#pragma once
#include <fstream>
#include <algorithm>
#include <vector>

#include "matrix.h"
#include "visibility.h"

//表示颜色的类型，颜色分量的取值位于区间[0,255]
struct Color {
    int r;
    int g;
    int b;
};
//定义颜色常量
const Color WHITE{ 255,255,255 }, BLACK{ 0,0,0 }, RED{ 255,0,0 }, GREEN{ 0,255,0 };

//画布类
class Canvas {
private:
    Color* pixels;
    float* z_buffer;
    Color foreground_color;
    int height, width;
public:
    //构造函数
    Canvas(int h, int w) :height(h), width(w) {
        pixels = new Color[h * w];
        z_buffer = new float[h * w];
        for (int i = 0; i < h * w; i++)
            z_buffer[i] = FLT_MAX;
        clear(WHITE);
    }
    ~Canvas() {
        delete [] pixels;
        delete [] z_buffer;
    }

    //绘制画布背景
    void clear(const Color& c) {
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                setPixel(x, y, c);
            }
    }
    //设置画布前景色
    void setColor(const Color& c) {
        foreground_color = c;
    }

    //设置指定像素的颜色
    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return;
        pixels[y * width + x] = c;
    }
    //设置指定像素的深度
    void setDepth(int x, int y, float d) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return;
        z_buffer[y * width + x] = d;
    }
    //获取指定像素的颜色
    Color getPixel(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return foreground_color;
        return pixels[y * width + x];
    }
    //获取指定像素的深度
    float getDepth(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return FLT_MAX;
        return z_buffer[y * width + x];
    }
    //输出图像到PPM文件，默认文件名为image.ppm
    void outputPPM() {
        ofstream out("imageGZ.ppm");
        //写入ppm文件前两行信息
        out << "P3\n" << width << ' ' << height << "\n255\n";
        //按行遍历像素数组，把像素颜色写入文件，原点在左下角，水平x方向，垂直y方向
        for (int y = height - 1; y >= 0; y--) {
            for (int x = 0; x < width; x++) {
                Color c = getPixel(x, y);
                out << c.r << ' ' << c.g << ' ' << c.b << '\n';
            }
        }
    }
    //输出图像到屏幕
    void outputScreen();
    //TODO：给定椭圆圆心坐标，长短轴长度，绘制椭圆
    void drawEllipse(int cx, int cy, int a, int b) {
        //使用SetPixel函数设置像素点颜色
    }

    //TODO：给定直线段两个端点的坐标，绘制直线
    void drawLine(const Vec& p1, const Vec& p2) {
        int x0 = p1.x() + 0.5,
            y0 = p1.y() + 0.5,
            x1 = p2.x() + 0.5,
            y1 = p2.y() + 0.5;
        //处理斜率绝对值大于1的情况
        bool steep = abs(int(y1 - y0)) > abs(int(x1 - x0));
        if (steep == true)
        {
            swap(x0, y0);
            swap(x1, y1);
        }
        //起点x坐标大于终点x坐标
        if (x0 > x1)
        {
            swap(x0, x1);
            swap(y0, y1);
        }
        //下降直线
        bool descend = y0 > y1;
        int offset = 2 * y0;
        if (descend == true)
        {
            y1 = offset - y1;
        }
        //初始化循环变量
        int dx = x1 - x0;
        int dy = y1 - y0;
        int dm = 2 * dy - dx;
        int dmp1 = 2 * dy;
        int dmp2 = 2 * (dy - dx);
        //循环遍历每一列，根据中点判别式符号选择离直线最近的点
        for (int x = x0, y = y0; x <= x1; x++)
        {
            int real_y = y;
            if (descend == true)
                real_y = offset - y;

            if (steep == true)
                setPixel(real_y, x, foreground_color);
            else
                setPixel(x, real_y, foreground_color);

            if (dm <= 0)    //中点在直线上方
                dm = dm + dmp1;
            else
            {
                dm = dm + dmp2;
                y++;
            }
        }
    }
    void drawBezier(const vector<Vec>& pnts) {
        //绘制控制多边形
        for (int i = 0; i < pnts.size() - 1; i++) {
            drawLine(pnts[i], pnts[i + 1]);
        }
        //对曲线进行采样，画出采样点
        int N = 100;
        for (int i = 0; i <= N; i++) {
            float t = float(i) / N;
            Vec p = deCasteljau(pnts, t);
            //画出点p
            setPixel(p.x()+0.5, p.y()+0.5, Color{ 0,255,0 });
        }
    }
    Vec deCasteljau(const vector<Vec>& P, float t) {
        int n = P.size() - 1;        
        //TODO：计算曲线上与参数t对应的点C
        vector<Vec> Q = P;
        for (int k = 1; k <= n; k++) {
            for (int i = 0; i <= n - k; i++) {
                Q[i] = (1 - t)*Q[i] + t * Q[i + 1];
            }
        }
        return Q[0];
    }
    void drawBSpline(const vector<Vec>& pnts, const vector<float>& knots, 
        int degree) {
        int n = pnts.size() - 1;
        for (int k = degree; k <= n; k++) {
            float dx = (knots[k + 1] - knots[k]) / 100;
            for (float x = knots[k]; x < knots[k + 1]; x += dx) {
                Vec p = deBoor(k, x, knots, pnts, degree);
                //画出p
            }
        }
    }
    //x：参数，在区间[t[k],t[k+1])内
    //k：x所在的参数区间的编号
    //t：节点向量
    //c：控制点数组
    //p：曲线的次数
    Vec deBoor(int k, float x, const vector<float>& t, 
        const vector<Vec>& c, int p) {
        //TODO：给出参数x，求对应点的坐标
    }
    //矩形
    //p4------p3
    //|        |
    //|        |qing
    //p1------p2
    void drawRect(const Vec& p1, const Vec& p2, const Vec& p3, const Vec& p4) {
        drawLine(p1, p2);
        drawLine(p2, p3);
        drawLine(p3, p4);
        drawLine(p4, p1);
    }
    int getW()const { return width; }
    int getH()const { return height; }
};
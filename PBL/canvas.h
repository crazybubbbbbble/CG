#pragma once
#include <fstream>
#include <algorithm>
#include <vector>

#include "matrix.h"
#include "visibility.h"

//��ʾ��ɫ�����ͣ���ɫ������ȡֵλ������[0,255]
struct Color {
    int r;
    int g;
    int b;
};
//������ɫ����
const Color WHITE{ 255,255,255 }, BLACK{ 0,0,0 }, RED{ 255,0,0 }, GREEN{ 0,255,0 };

//������
class Canvas {
private:
    Color* pixels;
    float* z_buffer;
    Color foreground_color;
    int height, width;
public:
    //���캯��
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

    //���ƻ�������
    void clear(const Color& c) {
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                setPixel(x, y, c);
            }
    }
    //���û���ǰ��ɫ
    void setColor(const Color& c) {
        foreground_color = c;
    }

    //����ָ�����ص���ɫ
    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return;
        pixels[y * width + x] = c;
    }
    //����ָ�����ص����
    void setDepth(int x, int y, float d) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return;
        z_buffer[y * width + x] = d;
    }
    //��ȡָ�����ص���ɫ
    Color getPixel(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return foreground_color;
        return pixels[y * width + x];
    }
    //��ȡָ�����ص����
    float getDepth(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return FLT_MAX;
        return z_buffer[y * width + x];
    }
    //���ͼ��PPM�ļ���Ĭ���ļ���Ϊimage.ppm
    void outputPPM() {
        ofstream out("imageGZ.ppm");
        //д��ppm�ļ�ǰ������Ϣ
        out << "P3\n" << width << ' ' << height << "\n255\n";
        //���б����������飬��������ɫд���ļ���ԭ�������½ǣ�ˮƽx���򣬴�ֱy����
        for (int y = height - 1; y >= 0; y--) {
            for (int x = 0; x < width; x++) {
                Color c = getPixel(x, y);
                out << c.r << ' ' << c.g << ' ' << c.b << '\n';
            }
        }
    }
    //���ͼ����Ļ
    void outputScreen();
    //TODO��������ԲԲ�����꣬�����᳤�ȣ�������Բ
    void drawEllipse(int cx, int cy, int a, int b) {
        //ʹ��SetPixel�����������ص���ɫ
    }

    //TODO������ֱ�߶������˵�����꣬����ֱ��
    void drawLine(const Vec& p1, const Vec& p2) {
        int x0 = p1.x() + 0.5,
            y0 = p1.y() + 0.5,
            x1 = p2.x() + 0.5,
            y1 = p2.y() + 0.5;
        //����б�ʾ���ֵ����1�����
        bool steep = abs(int(y1 - y0)) > abs(int(x1 - x0));
        if (steep == true)
        {
            swap(x0, y0);
            swap(x1, y1);
        }
        //���x��������յ�x����
        if (x0 > x1)
        {
            swap(x0, x1);
            swap(y0, y1);
        }
        //�½�ֱ��
        bool descend = y0 > y1;
        int offset = 2 * y0;
        if (descend == true)
        {
            y1 = offset - y1;
        }
        //��ʼ��ѭ������
        int dx = x1 - x0;
        int dy = y1 - y0;
        int dm = 2 * dy - dx;
        int dmp1 = 2 * dy;
        int dmp2 = 2 * (dy - dx);
        //ѭ������ÿһ�У������е��б�ʽ����ѡ����ֱ������ĵ�
        for (int x = x0, y = y0; x <= x1; x++)
        {
            int real_y = y;
            if (descend == true)
                real_y = offset - y;

            if (steep == true)
                setPixel(real_y, x, foreground_color);
            else
                setPixel(x, real_y, foreground_color);

            if (dm <= 0)    //�е���ֱ���Ϸ�
                dm = dm + dmp1;
            else
            {
                dm = dm + dmp2;
                y++;
            }
        }
    }
    void drawBezier(const vector<Vec>& pnts) {
        //���ƿ��ƶ����
        for (int i = 0; i < pnts.size() - 1; i++) {
            drawLine(pnts[i], pnts[i + 1]);
        }
        //�����߽��в���������������
        int N = 100;
        for (int i = 0; i <= N; i++) {
            float t = float(i) / N;
            Vec p = deCasteljau(pnts, t);
            //������p
            setPixel(p.x()+0.5, p.y()+0.5, Color{ 0,255,0 });
        }
    }
    Vec deCasteljau(const vector<Vec>& P, float t) {
        int n = P.size() - 1;        
        //TODO�����������������t��Ӧ�ĵ�C
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
                //����p
            }
        }
    }
    //x��������������[t[k],t[k+1])��
    //k��x���ڵĲ�������ı��
    //t���ڵ�����
    //c�����Ƶ�����
    //p�����ߵĴ���
    Vec deBoor(int k, float x, const vector<float>& t, 
        const vector<Vec>& c, int p) {
        //TODO����������x�����Ӧ�������
    }
    //����
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
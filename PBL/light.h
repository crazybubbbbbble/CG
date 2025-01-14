#pragma once
#include <iostream>
#include <cmath>
#include "matrix.h"


typedef Vec vec3;

class phonglighting {
public:

    //���캯��
    phonglighting(vec3& ambient, const vec3& light, const vec3& tex, float shininess)
        : ambientcolor(ambient), lightcolor(light), texcolor(tex), shininess(shininess) {}

    // �����������
    vec3 calculateambient()  {
        return ambientcolor.mutiply(texcolor);
    }

    // �����������
    vec3 calculatediffuse(const vec3& lightdir, const vec3& normal)  {
        float dotproduct = std::max(0.0f, lightdir.dot(normal));  //�����������������֮��ļнǳ���������ʮ�����迼��
        return (lightcolor * dotproduct).mutiply(texcolor);
    }

    // ���㾵�淴���
    vec3 calculatespecular(const vec3& lightdir, const vec3& viewdir, const vec3& normal) const {
        vec3 cur(2 * lightdir.dot(normal) * normal.operator[](0), 2 * lightdir.dot(normal) * normal.operator[](1), 2 * lightdir.dot(normal) * normal.operator[](2));
        vec3 reflection = lightdir - cur;
        float dotproduct = std::max(0.0f, reflection.dot(viewdir));
        return lightcolor * std::pow(dotproduct, shininess) * 0.05;  //����ks=0.05
    }
private:
    vec3 ambientcolor;
    vec3 lightcolor;
    vec3 texcolor;
    float shininess;
};


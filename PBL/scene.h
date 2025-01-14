#pragma once
#include "camera.h"
#include "canvas.h"
#include "OBJ_Loader.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include<float.h>
#include <vector>
#include <algorithm>
using namespace std;



typedef Vec Vec3;
//����
class Ray
{
private:
	Vec3 origin, derection;
public:
	//Ĭ�Ϲ��캯��
	Ray() {}
	//���캯��
	Ray(Vec3 ori, Vec dere) :origin(ori), derection(dere) {}
	Vec3 getpoint(float t) const
	{
		return origin + t * derection;
	}
	Vec3 getorigin()
	{
		return origin;
	}
	Vec3 getderection()
	{
		return derection;
	}

};


//������
class Texture
{
private:
	float* data;	//����ͼ�����ݣ����е�˳�򱣴����أ�ÿ������ռ��c��float
	int w, h, c;	//ͼ���ȡ��߶ȡ���ɫͨ��������RGBΪ3��RGBAΪ4��
public:
	//// Ĭ�Ϲ��캯��
	//Texture() : data(nullptr), w(0), h(0), c(0) {
	//	// ������ִ��Ĭ�ϵĳ�ʼ��������
	//}
	//���캯��
	Texture(const char* filename);
	//��������
	~Texture();

	//��������ھ�������ɫ
	Vec3 getNN(float u, float v) const;
	//˫���Բ�ֵ��������ɫ
	Vec3 bilinearInterpolation(float u, float v) const;
};
Texture::Texture(const char* filename)
{
	//��תy��
	stbi_set_flip_vertically_on_load(true);
	//��������ͼ��
	data = stbi_loadf(filename, &w, &h, &c, 0);
}
Texture::~Texture()
{
	//�ͷ���������
	stbi_image_free(data);
}

//TODO:��������ھ�������ɫ
//���룺��������u��v��ȡֵ��ΧΪ[0,1]
Vec3 Texture::getNN(float u, float v) const
{
	//ͼ��ռ�����
	if (u <= 0)u = 0;
	if (u >= 1)u = 1;
	if (v <= 0)v = 0;
	if (v >= 1)v = 1;

	int x = u * w, y = v * h;
	if (x >= w)x = w - 1;
	if (y >= h)y = h - 1;

	return Vec3{ data[y * w * c + x * c] ,data[y * w * c + x * c + 1],
		data[y * w * c + x * c + 2] };
}
//TODO��˫���Բ�ֵ��������ɫ
//���룺��������u��v��ȡֵ��ΧΪ[0,1]
Vec3 Texture::bilinearInterpolation(float u, float v) const
{
	Vec3 color;
	float x = u * (w - 1), y = v * (h - 1);

	return color;
}



Texture texture0("../hducg24/models/spot/textures/blanket1.png");
Texture texture1("../hducg24/models/spot/textures/candle1.png");
Texture texture2("../hducg24/models/spot/textures/curtain1.png");
Texture texture3("../hducg24/models/spot/textures/curtain1.png");
Texture texture4("../hducg24/models/spot/textures/floor1.jpg");
Texture texture5("../hducg24/models/spot/textures/fur1.png");
Texture texture6("../hducg24/models/spot/textures/magazine1.png");
Texture texture7("../hducg24/models/spot/textures/photo1.png");
Texture texture8("../hducg24/models/spot/textures/photo2.png");
Texture texture9("../hducg24/models/spot/textures/pillow1.png");
Texture texture10("../hducg24/models/spot/textures/pillow2.png");
Texture texture11("../hducg24/models/spot/textures/pillow3.png");
Texture texture12("../hducg24/models/spot/textures/pillow4.png");
Texture texture13("../hducg24/models/spot/textures/pillow5.png");
Texture texture14("../hducg24/models/spot/textures/plant1-1.png");
Texture texture15("../hducg24/models/spot/textures/plant1-2.png");
Texture texture16("../hducg24/models/spot/textures/plant2.png");
Texture texture17("../hducg24/models/spot/textures/pouf1.png");
Texture texture18("../hducg24/models/spot/textures/rug1.png");
Texture texture19("../hducg24/models/spot/textures/sofa1.png");
Texture texture20("../hducg24/models/spot/textures/sofa2.png");
Texture texture21("../hducg24/models/spot/textures/table1.png");
Texture texture22("../hducg24/models/spot/textures/window1-1.png");
Texture texture23("../hducg24/models/spot/textures/window1-2.png");
Texture texture24("../hducg24/models/spot/textures/window1-1.png");
Texture texture25("../hducg24/models/spot/textures/window1-2.png");
Texture texture26("../hducg24/models/spot/textures/wall1.png");
Texture texture27("../hducg24/models/spot/textures/wall1.png");
//Texture texture28("../hducg24/models/spot/textures/wall1.png");
//Texture texture29("../hducg24/models/spot/textures/wall1.png");
//Texture texture30("../hducg24/models/spot/textures/wall1.png");
//Texture texture31("../hducg24/models/spot/textures/floor1.jpg");

Texture* tex[28] = { &texture0,&texture1,&texture2,&texture3,&texture4 ,&texture5,
					&texture6,&texture7,&texture8,&texture9,&texture10,
					&texture11,&texture12,
					&texture13,&texture14,&texture15,&texture16 ,&texture17,&texture18,
					&texture19,&texture20,&texture21,&texture22 ,&texture23,&texture24,&texture25 ,
	&texture26,&texture27,/*&texture28,texture29 ,texture30,texture31,*/
};

//��������
class Triangle {
public:
	//Ĭ�Ϲ��캯��
	Triangle() {}
	//���캯��
	Triangle(const Vec3& A, const Vec3& B, const Vec3& C) {
		v[0] = A; v[1] = B; v[2] = C;
	}
	//�������ξ͵����任���ı����Ķ�������ֵ
	void transform(const Matrix4& m) {
		for (int i = 0; i < 3; i++) {
			v[i] = m * Vec4{ v[i] };
		}
		Matrix4 m1(m);
		m1 = m1.inverse();   //���������
		m1 = m1.transpose();  //����ת�þ���
		for (int i = 0; i < 3; i++) {
			n[i] = m1 * Vec4{ n[i] };
		}
	}
	//ȡ��i������
	Vec3 operator[](int i)const { return v[i]; }
	//���ö�������
	void setVertex(const Vec3& A, const Vec3& B, const Vec3& C) {
		v[0] = A; v[1] = B; v[2] = C;
	}
	//���ö��㷨��
	void setNormal(const Vec3& nA, const Vec3& nB, const Vec3& nC) {
		n[0] = nA; n[1] = nB; n[2] = nC;
	}
	Vec3 getNormal(int i)const { return n[i]; }
	//���ö�����������
	void setTexCoord(int i, float u, float v) {
		t[i][0] = u;
		t[i][1] = v;
	}
	//ȡ��i���������ɫ
	Vec getColor(int i)const { return c[i]; }
	//���õ�i���������ɫ
	void setColor(int i, const Vec3& color) { c[i] = color; }
	//ͨ��������������Ϊͬһ����ɫ�����������ɫ
	void setColor(const Vec3& color) { c[0] = c[1] = c[2] = color; }
	float getU(int i)const { return t[i][0]; }
	float getV(int i)const { return t[i][1]; }
private:
	//1.���������εĶ��㼰�������ԣ���ɫ��
	//�����ε���������
	Vec3 v[3];
	//�����ζ���ķ���
	Vec3 n[3];
	//�������ɫ
	Vec3 c[3];
	//�����Ӧ����������
	//t[i]��i��������������꣬u=t[i][0],b=t[i][1]��i=0,1,2
	float t[3][2];
};

//������
class Object {
public:
	//Object() : m(Matrix4()), kd(0.0f), ks(0.0f), tex(Texture()) {
	//	// ��ʼ��������ִ�������ʵ��Ĳ���
	//}
	//���ñ任���󣬲���ÿ��������ִ�б任
	void setTransform(const Matrix4& trans) {
		m = trans;
		for (int i = 0; i < triangles.size(); i++) {
			triangles[i].transform(m);
		}
	}
	//����objģ��
	void loadObj(string path,Vec3 s, Vec3 d) {
		objl::Loader Loader;
		bool loadout = Loader.LoadFile(path);

		ks = s;
		kd = kd;

		for (auto mesh : Loader.LoadedMeshes)
		{
			for (int i = 0; i < mesh.Vertices.size(); i += 3)
			{
				Vec A{ mesh.Vertices[i].Position.X, mesh.Vertices[i].Position.Y, mesh.Vertices[i].Position.Z },
					B{ mesh.Vertices[i + 1].Position.X, mesh.Vertices[i + 1].Position.Y, mesh.Vertices[i + 1].Position.Z },
					C{ mesh.Vertices[i + 2].Position.X, mesh.Vertices[i + 2].Position.Y, mesh.Vertices[i + 2].Position.Z };
				Vec nA{ mesh.Vertices[i].Normal.X, mesh.Vertices[i].Normal.Y, mesh.Vertices[i].Normal.Z },
					nB{ mesh.Vertices[i + 1].Normal.X, mesh.Vertices[i + 1].Normal.Y, mesh.Vertices[i + 1].Normal.Z },
					nC{ mesh.Vertices[i + 2].Normal.X, mesh.Vertices[i + 2].Normal.Y, mesh.Vertices[i + 2].Normal.Z };
				float tA[2]{ mesh.Vertices[i].TextureCoordinate.X, mesh.Vertices[i].TextureCoordinate.Y },
					tB[2]{ mesh.Vertices[i + 1].TextureCoordinate.X, mesh.Vertices[i + 1].TextureCoordinate.Y },
					tC[2]{ mesh.Vertices[i + 2].TextureCoordinate.X, mesh.Vertices[i + 2].TextureCoordinate.Y };

				Triangle t{ A,B,C };
				t.setNormal(nA, nB, nC);
				t.setTexCoord(0, tA[0], tA[1]);
				t.setTexCoord(1, tB[0], tB[1]);
				t.setTexCoord(2, tC[0], tC[1]);

				triangles.push_back(t);
			}
		}
	}
	float intersection(Ray ray, int& id)//����������������
	{
		float nearest = FLT_MAX;
		//todo��Χ��
		float t = FLT_MAX,b1,b2;
		for (int i = 0; i < triangles.size(); i++)
		{
			Vec3 E1 = triangles[i][1] - triangles[i][0];
			Vec3 E2 = triangles[i][2] - triangles[i][0];
			Vec3 S = ray.getorigin() - triangles[i][0];
			Vec3 S1 = ray.getderection() ^ E2;
			Vec3 S2 = S ^ E1;

			float coeff = 1.0f / (S1 * E1);
			t = coeff * (S2 * E2);
			b1 = coeff * (S1 * S);
			b2 = coeff * (S2 * ray.getderection());

			if (t >= 1e-5 && b1 >= 0 && b2 >= 0 && (1 - b1 - b2) >= 0 && nearest > t)//todo 
			{
				nearest = t;
				id = i;
			}
		}
		return nearest;		
	}
	//��ȡ�������б�
	const vector<Triangle>& getTriangles() const { return triangles; }
	Vec3 kd, ks;
//protected:
	//����ı任����
	Matrix4 m;
	//������������б�
	vector<Triangle> triangles;
	//Texture tex;
};

//����
class Scene {
public:
	Scene() :camera{ Vec{0,0,0},Vec{0,0,-1},Vec{0,1,0} } {
	}
	void setCamera(const Camera& camera) { this->camera = camera; }
	//�򳡾����������
	//���ݳ�����Χ���������������ʹ���������������������Ӿ�����
	void addObject(const Object& object) {
		//��ȡ������������񣬲���ӵ������������б���
		vector<Triangle> obj_triangles = object.getTriangles();
		triangles_world.insert(triangles_world.end(),
			obj_triangles.begin(), obj_triangles.end());

		//���㳡����Χ��
		float xmin, xmax, ymin, ymax, zmin, zmax;
		getBBox(xmin, xmax, ymin, ymax, zmin, zmax);
		float xspan = xmax - xmin;
		float yspan = ymax - ymin;
		float zspan = zmax - zmin;

		//������򳡾�����
		Vec lookat{ (xmin + xmax) / 2,(ymin + ymax) / 2,(zmin + zmax) / 2 };
		//�������z�������򣬵��������ľ���Ϊzmax + max(xspan,yspan)
		Vec eye = lookat + Vec{ 0,0, zmax + max(xspan,yspan) };
		Vec up{ 0,1,0 };
		//���������ͼ����
		camera.setView(eye, lookat, up);

		//�����������
		float f = max(xspan, yspan) + 2 * zspan;
		camera.setDepth(1, f);
	}
	//���Ƴ���
	void draw(Canvas& canvas, Object& object, const Texture* tex = nullptr);
	void drawTriangle(const Triangle& t, Canvas& canvas,
		const Texture* tex = nullptr);
	Camera camera;	//���
private:
	//���㳡����Χ��
	void getBBox(float& xmin, float& xmax, float& ymin, float& ymax,
		float& zmin, float& zmax) {
		Vec p = triangles_world[0][0];
		xmin = xmax = p.x();
		ymin = ymax = p.y();
		zmin = zmax = p.z();
		for (auto tri : triangles_world) {
			for (int i = 0; i < 3; i++) {
				Vec p = tri[i];
				if (xmin > p.x())xmin = p.x();
				if (xmax < p.x())xmax = p.x();
				if (ymin > p.y())ymin = p.y();
				if (ymax < p.y())ymax = p.y();
				if (zmin > p.z())zmin = p.z();
				if (zmax < p.z())zmax = p.z();
			}
		}
	}
	vector<Triangle> triangles_world;	//����������б���������������ϵ��
};

void Scene::draw(Canvas& canvas, Object& object, const Texture* tex)
{
	int w = canvas.getW();
	int h = canvas.getH();
	//canvas.clear(WHITE);

	//����ͶӰ���ڴ�С
	//��֤ͶӰ���ڱ������ӿڱ���һ��
	if (w > h)
		camera.setPerspective(-0.5 * w / h, 0.5 * w / h, -0.5, 0.5);
	else
		camera.setPerspective(-0.5, 0.5, -0.5 * h / w, 0.5 * h / w);


	//���Ƴ��������е�ÿ��������
	for (int i = 0; i < object.triangles.size(); i++)
	{
		Vec n1(object.triangles[i].getNormal(0));
		Vec n2(object.triangles[i].getNormal(1));
		Vec n3(object.triangles[i].getNormal(2));
		Vec v1(object.triangles[i][0] - camera.eye);
		Vec v2(object.triangles[i][1] - camera.eye);
		Vec v3(object.triangles[i][2] - camera.eye);
		v1.normalize(), v2.normalize(), v3.normalize();
		n1.normalize(), n2.normalize(), n3.normalize();
		if (n1.dot(v1) <= 1e-3 || n2.dot(v2) <= 1e-3 || n3.dot(v3) <= 1e-3)
			drawTriangle(object.triangles[i], canvas, tex);
	}
}

void Scene::drawTriangle(const Triangle& t, Canvas& canvas,
	const Texture* tex)
{
	Vec pn1 = camera.world2ndc(t[0], canvas.getW(), canvas.getH()),
		pn2 = camera.world2ndc(t[1], canvas.getW(), canvas.getH()),
		pn3 = camera.world2ndc(t[2], canvas.getW(), canvas.getH());
	if (
		(pn1.x() < -1 || pn1.x() > 1 || pn1.y() < -1 || pn1.y() > 1 || pn1.z() < -1 || pn1.z() > 1)
		&& (pn2.x() < -1 || pn2.x() > 1 || pn2.y() < -1 || pn2.y() > 1 || pn2.z() < -1 || pn2.z() > 1)
		&& (pn3.x() < -1 || pn3.x() > 1 || pn3.y() < -1 || pn3.y() > 1 || pn3.z() < -1 || pn3.z() > 1)
		)
	{
		return;
	}
	else
	{
		Vec p1 = camera.world2viewport(t[0], canvas.getW(), canvas.getH()),
			p2 = camera.world2viewport(t[1], canvas.getW(), canvas.getH()),
			p3 = camera.world2viewport(t[2], canvas.getW(), canvas.getH());
		
		int xmin = min(min(p1.x(), p2.x()), p3.x()) + 0.5;
		if (xmin < 0)xmin = 0;
		int xmax = max(max(p1.x(), p2.x()), p3.x()) + 0.5;
		if (xmax > canvas.getW() - 1)xmax = canvas.getW() - 1;
		int ymin = min(min(p1.y(), p2.y()), p3.y()) + 0.5;
		if (ymin < 0)ymin = 0;
		int ymax = max(max(p1.y(), p2.y()), p3.y()) + 0.5;
		if (ymax > canvas.getH() - 1)ymax = canvas.getH() - 1;

		//Ԥ���㶥����ȵĵ���
		float _p1z = 1 / p1.z(), _p2z = 1 / p2.z(), _p3z = 1 / p3.z();
		//���㷨��
		Vec p1n = t.getNormal(0), p2n = t.getNormal(1), p3n = t.getNormal(2);
		//��ȡ�������������
		float u1 = t.getU(0), u2 = t.getU(1), u3 = t.getU(2);
		float v1 = t.getV(0), v2 = t.getV(1), v3 = t.getV(2);
		//���������ΰ�Χ���ڵ����ص�
		for (int x = xmin; x <= xmax; x++) {
			for (int y = ymin; y <= ymax; y++) {
				//�������ص����������
				Vec bc = baryCentri(p1, p2, p3, Vec{ float(x),float(y),1 });
				//�������������жϵ��Ƿ�����������
				if (0 <= bc[0] && bc[0] <= 1 &&
					0 <= bc[1] && bc[1] <= 1 &&
					0 <= bc[2] && bc[2] <= 1) {
					//��ֵ�õ����
					float z = 1 / (bc[0] * _p1z + bc[1] * _p2z + bc[2] * _p3z);
					//�ο����ϣ�
					//https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/perspective-correct-interpolation-vertex-attributes.html
					//TODO:�����ֵ
					//Vec n = z * (p1n * _p1z * bc[0] + p2n * _p2z * bc[1] + 
					//	p3n * _p3z * bc[2]);
					//n.normalize();
					//TODO:�������������ֵ
					//TODO:���������ֵ
					float u = z * (u1 * _p1z * bc[0] + u2 * _p2z * bc[1] +
						u3 * _p3z * bc[2]);
					float v = z * (v1 * _p1z * bc[0] + v2 * _p2z * bc[1] +
						v3 * _p3z * bc[2]);
					//TODO:��ȡ������ɫ
					Vec3 tex_color = tex->getNN(u, v);
					//Vec3 tex_color_b = tex->bilinearInterpolation(u, v);
					//TODO:Phong������ɫ����		
					Color c{ tex_color[0] * 255,tex_color[1] * 255,tex_color[2] * 255 };
					if (z < canvas.getDepth(x, y)) {
						canvas.setPixel(x, y, c);
						canvas.setDepth(x, y, z);
					}
				}
			}
		}
	}
}

//������
class Cube :public Object {
public:
	//����һ����λ�����壬������������������
	Cube() {
		Vec p1{ 0,0,0 }, p2{ 1,0,0 }, p3{ 1,1,0 }, p4{ 0,1,0 },
			p5{ 0,0,1 }, p6{ 1,0,1 }, p7{ 1,1,1 }, p8{ 0,1,1 };
		//����
		triangles.push_back(Triangle{ p1,p3,p2 });
		triangles.push_back(Triangle{ p1,p4,p3 });
		//ǰ��
		triangles.push_back(Triangle{ p5,p6,p7 });
		triangles.push_back(Triangle{ p5,p7,p8 });
		//����
		triangles.push_back(Triangle{ p1,p2,p6 });
		triangles.push_back(Triangle{ p1,p6,p5 });
		//����
		triangles.push_back(Triangle{ p4,p7,p3 });
		triangles.push_back(Triangle{ p4,p8,p7 });
		//����
		triangles.push_back(Triangle{ p1,p5,p4 });
		triangles.push_back(Triangle{ p5,p8,p4 });
		//����
		triangles.push_back(Triangle{ p3,p6,p2 });
		triangles.push_back(Triangle{ p3,p7,p6 });
		//����ÿ�������ɫ
		for (int i = 0; i < 12; i++) {
			triangles[i].setColor(Vec{ float(i / 2) / 6,0,0 });
		}
	}
};

//��
class Sphere :public Object {
public:
	//TODO:����������ԭ��ĵ�λ������������������
	//�ο����ϣ�https://blog.csdn.net/hitzsf/article/details/128632607
	Sphere() {
		//��1��������һ��������ԭ�����������
		Vec p1{ 0,0,1 }, p2{ 0,0.942809,-0.333333 },
			p3{ -0.816497,-0.471405,-0.333333 }, p4{ 0.816497,-0.471405,-0.333333 };
		Triangle t1{ p1,p2,p3 }, t2{ p1,p3,p4 },
			t3{ p1,p4,p2 }, t4{ p2,p4,p3 };
		//��2������ÿ�������ν���ϸ��
		divideTriangle(t1, 4);
		divideTriangle(t2, 4);
		divideTriangle(t3, 4);
		divideTriangle(t4, 4);
	}
private:
	void divideTriangle(Triangle t, int n) {
		if (n > 0) {
			Vec v1 = 0.5 * (t[0] + t[1]);
			Vec v2 = 0.5 * (t[0] + t[2]);
			Vec v3 = 0.5 * (t[1] + t[2]);
			v1.normalize();
			v2.normalize();
			v3.normalize();
			divideTriangle(Triangle{ t[0], v1, v2 }, n - 1);
			divideTriangle(Triangle{ t[1], v3, v1 }, n - 1);
			divideTriangle(Triangle{ t[2], v2, v3 }, n - 1);
			divideTriangle(Triangle{ v1, v3, v2 }, n - 1);
		}
		else {
			t.setNormal(t[0], t[1], t[2]);
			triangles.push_back(t);
		}
	}
};


//��Դ��
class Light
{
private:
	Vec3 position;
	Vec3 color;
	float strength;
public:
	Light(Vec3 pos = Vec3(0, 0, 0), Vec col = Vec3(1,1,1),float str = 0) :position(pos), color(col),strength(str) {}
	Vec3 getpostion()
	{
		return position;
	}
	float getstrength()
	{
		return strength;
	}
	Vec3 getcolor()
	{
		return color;
	}
};

Vec3 bilinearInterpolation(const Triangle& triangle, const Vec3& intersectionPoint, int id) {
	// ��ȡ�������������
	Vec3 t0 = triangle[0];
	Vec3 t1 = triangle[1];
	Vec3 t2 = triangle[2];

	//��ȡ�������������
	float u1 = triangle.getU(0), u2 = triangle.getU(1), u3 = triangle.getU(2);
	float v1 = triangle.getV(0), v2 = triangle.getV(1), v3 = triangle.getV(2);

	float areaABC = ((t1 - t0).cross(t2 - t0)).dot((t1 - t0).cross(t2 - t0));
	float alpha = ((t1 - intersectionPoint).cross(t2 - intersectionPoint)).dot((t1 - intersectionPoint).cross(t2 - intersectionPoint)) / areaABC;
	float beta = ((t2 - intersectionPoint).cross(t0 - intersectionPoint)).dot((t2 - intersectionPoint).cross(t0 - intersectionPoint)) / areaABC;
	float gamma = 1 - alpha - beta;

	// ʹ�������������uv˫���Բ�ֵ
	float u = (alpha * u1) + (beta * u2) + (gamma * u3);
	float v = (alpha * v1) + (beta * v2) + (gamma * v3);

	//����getColor()�������255����ע���Ͼ����¾�
	Vec3 color = alpha * triangle.getColor(0) / 255 + beta * triangle.getColor(1) / 255 + gamma * triangle.getColor(2) / 255;
	//Texture temp = tex[id];
	//TODO:��ȡ������ɫ
	Vec3 tex_color = tex[id]->getNN(u, v);
	//Vec3 tex_color_b = tex->bilinearInterpolation(u, v);
	//TODO:Phong������ɫ����		
	//Vec3 c{ tex_color[0] * 255,tex_color[1] * 255,tex_color[2] * 255 };
	Vec3 c{ tex_color[0],tex_color[1],tex_color[2] };
	return c;
}

Vec3 interpolateNormal(const Triangle& triangle, const Vec3& intersectionPoint) {
	// �洢���㴦����������
	Vec3 barycentricCoords;

	// ������������Ĺ��������ڽ����������ε����������������ϵ��
	// v0��v1 �� v2 �ֱ��Ǵӵ�һ������ָ���������������Լ������������
	Vec3 v0 = triangle[1] - triangle[0];
	Vec3 v1 = triangle[2] - triangle[0];
	Vec3 v2 = intersectionPoint - triangle[0];

	// �ڻ����㣬����������������ķ��Ӳ���
	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);

	float denom = d00 * d11 - d01 * d01;

	// ��������ļ���
	// ���������εĶ���ͽ����������ϵ�����������ڻ�ֵ������������ꡣ��Щ���㿼������������ͽ����λ�ù�ϵ��
	barycentricCoords[1] = (d11 * d20 - d01 * d21) / denom;
	barycentricCoords[2] = (d00 * d21 - d01 * d20) / denom;
	barycentricCoords[0] = 1.0f - barycentricCoords[1] - barycentricCoords[2];

	// �Է��������м�Ȩ���
	Vec3 interpolatedNormal = triangle.getNormal(0) * barycentricCoords[0] +
		triangle.getNormal(1) * barycentricCoords[1] +
		triangle.getNormal(2) * barycentricCoords[2];

	return interpolatedNormal;
}


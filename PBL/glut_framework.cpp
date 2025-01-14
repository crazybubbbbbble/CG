#define GLUT_DISABLE_ATEXIT_HACK
#include "scene.h"
#include "canvas.h"
#include "GLUT.H"
#include "camera.h"
#include"light.h"

#define _USE_MATH_DEFINES 
#include<math.h>
#include<stdio.h>
const int deep = 15;//递归深度
const int num = 2;
const int lambda = 0.01;
Light light;
int state = 0;

Object obj[2];


int object_num = 2;
int texture_num = 2;


//绘制场景
//视口大小
int viewport_width = 800, viewport_height = 600;
float xmove = 0;
float ymove = -1;
float zmove = 0;
float ex = 6.88577, ey = 1.84827, ez = 18.502;
Canvas canvasTacing(viewport_height, viewport_width);


Scene scene;

float Clamp(double x) {
	return x < 0 ? 0 : x > 1 ? 1 : x;               // CLAMP function,  0 < x < 1
}

float Max(float x, float y)
{
	return x > y ? x : y;
}

float Random() {
	return (float)rand() / (float)RAND_MAX;
}
//用于光追的求交判断,利用Moller-Trumbore射线-三角形求交算法
bool rayTriangleIntersect(Ray ray, float& t, int &idt, int &ido)
{
	float temp = 0;
	int tempid;
	t = FLT_MAX;
	for (int i = 0; i < num; i++)
	{
		temp = obj[i].intersection(ray, tempid);
		if (temp > 0 && temp < t)
		{
			t = temp;
			idt = tempid;
			ido = i;
		}
	}
	return t<FLT_MAX&&t>1e-5;//t<无穷大
}

//追踪和着色
Vec3 TracingAndShading(Ray ray, int depth)
{
	float t;
	int idt = 0, ido = 0;
	depth++;
	if (depth > deep)
	{
		return Vec3(0.0, 0.0, 0.0);//超过追踪深度，不碰到光源返回背景色
	}
	if (!rayTriangleIntersect(ray, t, idt, ido))//求交失败，返回背景色
		return Vec3(0, 0, 0);
	if (ray.getorigin() + t * ray.getderection() == light.getpostion())//追踪到光源，返回光源颜色
		return bilinearInterpolation(obj[ido].getTriangles()[idt], ray.getorigin() + t * ray.getderection(),ido);//获取与三角形求交到的交点颜色（纹理颜色能不能获取？）
		
	Vec3 p = ray.getorigin() + t * ray.getderection();//交点坐标
	Vec3 n = interpolateNormal(obj[ido].getTriangles()[idt],p);//求交点的法向量
	Vec3 n1 = n.dot(ray.getderection()) < 0 ? n : n * -1;//判断射线从射入还是射出，射入的话取n方向否则取n反方向
	
	//进行漫反射（折射也算进来了？）
	float r1 = 2 * M_PI * Random();
	float r2 = Random();

	Vec3 w = n1;
	Vec3 u = (fabs(w.x()) > 0.1 ? Vec3(0, 1, 0) : Vec3(1, 1, 1) ^ w);
	u.normalize();
	Vec3 v = w ^ u;
	Vec3 d = u * cos(r1) * sqrt(r2) + v * sin(r1) * sqrt(r2) + w * sqrt(1 - r2);
	d.normalize();//生成随机漫反射方向
	Vec3 h = (ray.getderection() * -1 + d);//原点光线和反射光线的平分线矢量
	h.normalize();
	Ray reflect_ray(p, d);
	Vec3 reflect_color = TracingAndShading(reflect_ray, depth);
	if (reflect_color.x() == -1.0)
		return reflect_color;
	Vec3 color = bilinearInterpolation(obj[ido].getTriangles()[idt], p,ido);	

	//Vec ambient(0, 0, 0);
	//phonglighting phong1(ambient, light.getcolor(), color, 8);
	//Vec color1 = phong1.calculateambient() + phong1.calculatediffuse(ray.getderection(), n) ;

	color = color * Max(0.0, n.dot(d)) + color.mutiply(obj[ido].ks) + reflect_color * lambda;
	/*color.normalize();*/
	return color;//根据公式计算返回颜色
}

Vec3* RayTracing() {
	int height = viewport_height;
	int width = viewport_width;
	unsigned seed = time(0);
	srand(seed);
	Vec3* image = new Vec3[width * height];
	int count = 0;
	int all = height * width;
	float jindu = 0.0;

	scene.camera.setView(Vec{ xmove + ex,ymove + ey, ez-20 }, Vec{ 0 ,0 ,0 }, Vec{ 0,1,0 });
	scene.camera.setView(Vec{ xmove + ex, ey+4,zmove + ez }, Vec{ 0 ,0 ,0 }, Vec{ 0,1,0 });
	/* main ray tracing loop */
	for (int row = 0; row < height; row++) {                     // looping over image rows//79.43
		for (int col = 0; col < width; col++) {                 // looping over image colomns
			int index = (height - row - 1) * width + col;              // map 2D pixel location to 1D vector
			Vec3 pixel_color(0.0, 0.0, 0.0);
			Vec3 pixel_position = Vec3(row, col, scene.camera.Getn());
			Vec3 pixel_real_position = scene.camera.viewport2world(pixel_position, width, height);
			Vec3 rayep = pixel_real_position - scene.camera.eye;
			rayep.normalize();
			Ray ray(scene.camera.eye, rayep);
			int depth = 0;
			pixel_color = TracingAndShading(ray, depth);
			//			for (int srow = 0; srow < 2; srow++) {              // looping over 2x2 subsample rows
			//				for (int scol = 0; scol < 2; scol++) {         // looping over 2x2 subsample colomns
			//					Vector3 offset(double(row) + double(srow), double(col) + double(scol), 0.0);
			//					Ray camera(offset, direction);
			//					int depth = 0;
			//
			//					pixel_color = pixel_color + TracingAndShading(camera, depth) * 0.25;
			//				}
			//			}
			//image[index] = image[index] + Vec3(Clamp(pixel_color[0]), Clamp(pixel_color[1]), Clamp(pixel_color[2]));
			image[index] = Vec3(Clamp(pixel_color[0]), Clamp(pixel_color[1]), Clamp(pixel_color[2]));
			count++;
			jindu = ((float)count / all)*100;
			system("cls");
			printf_s("%.3f", jindu);
		}
	}
	return image;

}

void Canvas::outputScreen()
{
	glBegin(GL_POINTS);
	//按行遍历像素数组，用opengl画点函数绘制像素
	//原点在左下角，水平x方向，垂直y方向
	for (int y = height - 1; y >= 0; y--) {
		for (int x = 0; x < width; x++) {
			Color c = getPixel(x, y);
			glColor3f(float(c.r) / 255, float(c.g) / 255, float(c.b) / 255);
			glVertex3f(x, y, getDepth(x, y));
		}
	}
	glEnd();
}

//绘制内容
void display(void)
{
	//设置画布背景色
	glClearColor(1.f, 1.f, 1.f, 0.f);
	glClear(GL_COLOR_BUFFER_BIT);
	if (state == 0)
	{
		//绘制场景
		Canvas canvas(viewport_height, viewport_width);
		Color c;
		c.r = 1;
		c.g = 1;
		c.b = 1;
		canvas.clear(c);
		Texture texture("F:\\visual stdio\\hducg24\\models\\b536bae2155826be5b6d07aeb552df5.png");  //纹理文件
		for (int i = 0; i < texture_num; i++) {
			scene.draw(canvas, obj[i], &texture);
		}

		for (int i = 0; i < object_num; i++) {
			scene.addObject(obj[i]);
		}
		canvas.outputScreen();
		
	}
	else
	{
		canvasTacing.outputScreen();
	}
	
	//canvas.outputPPM();
	cout << scene.camera.eye.operator[](0) << " " << scene.camera.eye.operator[](1) << " " << scene.camera.eye.operator[](2) << endl;
	cout << scene.camera.at.operator[](0) << " " << scene.camera.at.operator[](1) << " " << scene.camera.at.operator[](2);
	//刷新帧缓存
	glFlush();
}

//键盘交互事件
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'a':
	case 'A':
		//从右边看
	{
		
		ymove += 0.3;
		scene.camera.setView(Vec{ 4.6,2,2.3 }, Vec{ 0,0,ymove}, Vec{0,1,0});
		cout << "右视图" << endl;
		glutPostRedisplay();
		break;
	}
	//从后边看
	case 'd':
	case 'D':
	{
		 ymove-=0.3;
		//Camera camera(Vec{xmove,0,0}, Vec{ 0,0,-1 }, Vec{ 0,1,0 });
		scene.camera.setView(Vec{ 4.6,2,2.3 }, Vec{ 0,0,ymove }, Vec{ 0,1,0 });
		cout << "后视图" << endl;
		glutPostRedisplay();
		break;
	}
	case 'w':
	case 'W':
	{
		ymove+=0.4;
		//Camera camera(Vec{xmove,0,0}, Vec{ 0,0,-1 }, Vec{ 0,1,0 });
		scene.camera.setView(Vec{ 4.6,ymove,2.3 }, Vec{ 0 ,0 ,0 }, Vec{ 0,1,0 });
		cout << "y" << ymove << endl;
		glutPostRedisplay();
		break;
	}
	case 's':
	case 'S':
	{
		ymove-=0.4;
		//Camera camera(Vec{xmove,0,0}, Vec{ 0,0,-1 }, Vec{ 0,1,0 });
		scene.camera.setView(Vec{ 4.6,ymove,2.3 }, Vec{ 0 ,0 ,0 }, Vec{ 0,1,0 });
		cout << "y：" << ymove << endl;
		glutPostRedisplay();
		break;
	}
	
	
	case 'z':
	case 'Z':
	{
		xmove++;
		//Camera camera(Vec{xmove,0,0}, Vec{ 0,0,-1 }, Vec{ 0,1,0 });
		scene.camera.setView(Vec{ xmove ,2,2.3}, Vec{ 0 ,0 ,0 }, Vec{ 0,1,0 });
		cout << "z:" << zmove << endl;
		glutPostRedisplay();
		break;
	}
	case 'x':
	case 'X':
	{
		xmove--;
		//Camera camera(Vec{xmove,0,0}, Vec{ 0,0,-1 }, Vec{ 0,1,0 });
		scene.camera.setView(Vec{ xmove ,2,2.3 }, Vec{ 0 ,0 ,0 }, Vec{ 0,1,0 });
		cout << "z:" << zmove << endl;
		glutPostRedisplay();
		break;
	}
	case 'l':
	case 'L':
	{
		state = 1;
		Vec3* img = RayTracing();
		for (int row = 0; row < viewport_height; row++) {                     // looping over image rows
			for (int col = 0; col < viewport_width; col++) {                 // looping over image colomns
				int index = (viewport_height - row - 1) * viewport_width + col;
				Color color;
				color.r = (int)(img[index].operator[](0)*255);
				color.g = (int)(img[index].operator[](1)*255);
				color.b = (int)(img[index].operator[](2)*255);
				canvasTacing.setPixel(row, col, color);
			}
		}
		canvasTacing.outputPPM();
		glutPostRedisplay();
		cout << "over" << endl;
		break;
	}
	case 'g':
	case 'G':
	{
		state = 0;
		break;
	}
	case 27:
		exit(0);
		break;
	}
}

//鼠标交互事件
void mouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			xmove += 1;
			glutPostRedisplay();
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			ymove += 1;
			glutPostRedisplay();
		}
		break;
	default:
		break;
	}
}

//投影方式、modelview方式等设置
void reshape(int w, int h)
{
	viewport_width = w, viewport_height = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);//视口大小
	glMatrixMode(GL_PROJECTION);//设置投影模式以及视景体大小
	glLoadIdentity();
	//设置opengl的投影方式为正交投影
	//投影平面大小与视口大小一致
	//用glVertex2f模拟画像素
	glOrtho(0, w, 0, h, -200, 200);
	//将opengl的视图变换矩阵为单位矩阵
	//我们在Camera类中自己实现视图、投影和视口变换
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
//初始化场景
void initScene()
{
	
	obj[0].loadObj("F:\\visual stdio\\hducg24\\models\\场景.obj", Vec{ 0.0,0.0,0.0 }, Vec{ 1,1,1 });
	//obj[0].loadObj("D://计图大作业/hducg24/models/spot/models/spot/spot_triangulated.obj", Vec{ 0.0,0.0,0.0 }, Vec{ 0.64,0.64,0.64 });
	//obj[0].loadObj("D://计图大作业/hducg24/models/rock/rock.obj", Vec{ 0,0,0 }, Vec{ 0.2,0.2,0.2 });
	
	
	//for (int i = 0; i < 26; i++)
	//{
	//	obj[i].tex = tex[i];
	//}

	Matrix4 m;
	m.setRotate(120, Vec{ 0,1,0 });
	obj[0].setTransform(m);
	
	
	
}
//主调函数
int main(int argc, char** argv)
{
	//初始化场景
	initScene();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB);
	glutInitWindowSize(viewport_width, viewport_height);
	glutInitWindowPosition(50, 50);
	glutCreateWindow("HelloWorld");

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);

	glutMainLoop();
	return 0;
}


#pragma once
#include "matrix.h"
#include <Eigen/Dense>

typedef Vec Vec3;

class Camera {
public:
	//�������λ�ã��������ĵ㣬�Ϸ��򣬹�����ͼ�任����
	//�ο����ϣ�https://www.songho.ca/opengl/gl_camera.html#lookat
	Camera(const Vec3& eye, const Vec3& at, const Vec3& up) {
		mview.setView(eye, at, up);
		Eigen::Matrix4f matrix;
		matrix << mview.get(0, 0), mview.get(0, 1), mview.get(0, 2), mview.get(0, 3),
			mview.get(1, 0), mview.get(1, 1), mview.get(1, 2), mview.get(1, 3),
			mview.get(2, 0), mview.get(2, 1), mview.get(2, 2), mview.get(2, 3),
			mview.get(3, 0), mview.get(3, 1), mview.get(3, 2), mview.get(3, 3);
		Eigen::Matrix4f innversematrix = matrix.inverse();
		fmview.set(0, 0, innversematrix(0, 0)), fmview.set(0, 1, innversematrix(0, 1)), fmview.set(0, 2, innversematrix(0, 2)), fmview.set(0, 3, innversematrix(0, 3));
		fmview.set(1, 0, innversematrix(1, 0)), fmview.set(1, 1, innversematrix(1, 1)), fmview.set(1, 2, innversematrix(1, 2)), fmview.set(1, 3, innversematrix(1, 3));
		fmview.set(2, 0, innversematrix(2, 0)), fmview.set(2, 1, innversematrix(2, 1)), fmview.set(2, 2, innversematrix(2, 2)), fmview.set(2, 3, innversematrix(2, 3));
		fmview.set(3, 0, innversematrix(3, 0)), fmview.set(3, 1, innversematrix(3, 1)), fmview.set(3, 2, innversematrix(3, 2)), fmview.set(3, 3, innversematrix(3, 3));
		//setPerspective(-0.5, 0.5, -0.5, 1, 1, 6);
		//cout <<"��ͼ�任����" <<endl<< mview << endl;
		this->eye = eye;
	}

	void setView(const Vec3& eye, const Vec3& at, const Vec3& up) {
		mview.setView(eye, at, up);
		Eigen::Matrix4f matrix;
		matrix << mview.get(0, 0), mview.get(0, 1), mview.get(0, 2), mview.get(0, 3),
			mview.get(1, 0), mview.get(1, 1), mview.get(1, 2), mview.get(1, 3),
			mview.get(2, 0), mview.get(2, 1), mview.get(2, 2), mview.get(2, 3),
			mview.get(3, 0), mview.get(3, 1), mview.get(3, 2), mview.get(3, 3);
		Eigen::Matrix4f innversematrix = matrix.inverse();
		fmview.set(0, 0, innversematrix(0, 0)), fmview.set(0, 1, innversematrix(0, 1)), fmview.set(0, 2, innversematrix(0, 2)), fmview.set(0, 3, innversematrix(0, 3));
		fmview.set(1, 0, innversematrix(1, 0)), fmview.set(1, 1, innversematrix(1, 1)), fmview.set(1, 2, innversematrix(1, 2)), fmview.set(1, 3, innversematrix(1, 3));
		fmview.set(2, 0, innversematrix(2, 0)), fmview.set(2, 1, innversematrix(2, 1)), fmview.set(2, 2, innversematrix(2, 2)), fmview.set(2, 3, innversematrix(2, 3));
		fmview.set(3, 0, innversematrix(3, 0)), fmview.set(3, 1, innversematrix(3, 1)), fmview.set(3, 2, innversematrix(3, 2)), fmview.set(3, 3, innversematrix(3, 3));
		this->eye = eye;
	}
	void setDepth(float n, float f) {
		this->n = n;
		this->f = f;
	}

	//��������ͶӰ����
	void setOrtho(float l, float r, float b, float t, float n, float f) {
		mproj.setOrtho(l, r, b, t, n, f);
		this->n = n;
		this->f = f;
	}
	//����͸��ͶӰ����
	//�ο����ϣ�https://www.songho.ca/opengl/gl_projectionmatrix.html
	void setPerspective(float l, float r, float b, float t, float n, float f) {
		mproj.setPerspective(l, r, b, t, n, f);
		Eigen::Matrix4f matrix;
		matrix << mproj.get(0, 0), mproj.get(0, 1), mproj.get(0, 2), mproj.get(0, 3),
			mproj.get(1, 0), mproj.get(1, 1), mproj.get(1, 2), mproj.get(1, 3),
			mproj.get(2, 0), mproj.get(2, 1), mproj.get(2, 2), mproj.get(2, 3),
			mproj.get(3, 0), mproj.get(3, 1), mproj.get(3, 2), mproj.get(3, 3);
		Eigen::Matrix4f innversematrix = matrix.inverse();
		fmproj.set(0, 0, innversematrix(0, 0)), fmproj.set(0, 1, innversematrix(0, 1)), fmproj.set(0, 2, innversematrix(0, 2)), fmproj.set(0, 3, innversematrix(0, 3));
		fmproj.set(1, 0, innversematrix(1, 0)), fmproj.set(1, 1, innversematrix(1, 1)), fmproj.set(1, 2, innversematrix(1, 2)), fmproj.set(1, 3, innversematrix(1, 3));
		fmproj.set(2, 0, innversematrix(2, 0)), fmproj.set(2, 1, innversematrix(2, 1)), fmproj.set(2, 2, innversematrix(2, 2)), fmproj.set(2, 3, innversematrix(2, 3));
		fmproj.set(3, 0, innversematrix(3, 0)), fmproj.set(3, 1, innversematrix(3, 1)), fmproj.set(3, 2, innversematrix(3, 2)), fmproj.set(3, 3, innversematrix(3, 3));
		this->n = n;
		this->f = f;
		//cout << "͸��ͶӰ����" << endl << mproj << endl;
	}

	void setPerspective(float l, float r, float b, float t) {
		mproj.setPerspective(l, r, b, t, n, f);
		Eigen::Matrix4f matrix;
		matrix << mproj.get(0, 0), mproj.get(0, 1), mproj.get(0, 2), mproj.get(0, 3),
			mproj.get(1, 0), mproj.get(1, 1), mproj.get(1, 2), mproj.get(1, 3),
			mproj.get(2, 0), mproj.get(2, 1), mproj.get(2, 2), mproj.get(2, 3),
			mproj.get(3, 0), mproj.get(3, 1), mproj.get(3, 2), mproj.get(3, 3);
		Eigen::Matrix4f innversematrix = matrix.inverse();
		fmproj.set(0, 0, innversematrix(0, 0)), fmproj.set(0, 1, innversematrix(0, 1)), fmproj.set(0, 2, innversematrix(0, 2)), fmproj.set(0, 3, innversematrix(0, 3));
		fmproj.set(1, 0, innversematrix(1, 0)), fmproj.set(1, 1, innversematrix(1, 1)), fmproj.set(1, 2, innversematrix(1, 2)), fmproj.set(1, 3, innversematrix(1, 3));
		fmproj.set(2, 0, innversematrix(2, 0)), fmproj.set(2, 1, innversematrix(2, 1)), fmproj.set(2, 2, innversematrix(2, 2)), fmproj.set(2, 3, innversematrix(2, 3));
		fmproj.set(3, 0, innversematrix(3, 0)), fmproj.set(3, 1, innversematrix(3, 1)), fmproj.set(3, 2, innversematrix(3, 2)), fmproj.set(3, 3, innversematrix(3, 3));
		//cout << "͸��ͶӰ����" << endl << mproj << endl;
	}
	//��������תΪ�������
	Vec4 world2eye(const Vec3& pw) {
		return mview * Vec4{pw};
	}
	//�������תΪ�ü�����
	//x=pe.x*2n/(r-l)+pe.z*(r+l)/(r-l)
	//y=pe.y*2n/(t-b)+pe.z*(t+b)/(t-b)
	//z=-pe.z*(f+n)/(f-n)-2fn/(f-n)
	//w=-pe.z
	Vec4 eye2clip(const Vec4& pe) {
		return mproj * pe;
	}	
	//�ü�����תΪ��׼�豸����
	//x: [l,r]-->[-1,1],x=-pe.x/pe.z*2n/(r-l)-(r+l)/(r-l)
	//y: [b,t]-->[-1,1], y=-pe.y/pe.z*2n/(t-b)-(t+b)/(t-b)
	//z: [-n,-f]-->[-1,1], z=(f+n)/(f-n)+2fn/pe.z/(f-n)
	Vec3 clip2ndc(const Vec4& pc) {
		return Vec3{ pc.x() / pc.w(),pc.y() / pc.w(),pc.z() / pc.w() };
	}
	//��׼�豸����תΪ�ӿ�����
	//�ο����ϣ�https://www.songho.ca/opengl/gl_transform.html
	//x: [-1,1]-->[0,w]
	//y: [-1,1]-->[0,h]
	//z: [-1,1]-->[n,f]
	Vec3 ndc2viewport(const Vec3& pn, int w, int h) {
		return Vec3{ (pn.x() + 1) * w / 2,(pn.y() + 1) * h / 2,
			-2 * f * n / (pn.z() * (f - n) - (f + n)) };
			//pn.z() * (f - n) / 2 + (f + n) / 2 };
	}
	//��������任���ӿ�����
	Vec3 world2viewport(const Vec& pw, int w, int h) {
		Vec4 pc = mproj * mview * Vec4{ pw };
		Vec3 pn{ pc.x() / pc.w(),pc.y() / pc.w(),pc.z() / pc.w() };
		return Vec3{ (pn.x() + 1) * w / 2,(pn.y() + 1) * h / 2,
			-2 * f * n / (pn.z() * (f - n) - (f + n)) };
	}
	//�ӿ�����仯����������
	Vec3 viewport2world(const Vec& pv, int w, int h) {
		//Vec3 pc{ pw.x() * 2 / w - 1,pw.y() * 2 / h - 1,((-2 * f * n) / pw.z() + f + n) / (f - n) };
		Vec4 pc{ pv.x() * 2 / w - 1,pv.y() * 2 / h - 1,-n,n };
		Vec4 pn = fmview * fmproj * Vec4{ pc };
		//Vec3 result{ pn.x() / pn.w(),pn.y() / pn.w(),pn.z() / pn.w() };
		Vec3 result{ pn.x(),pn.y() ,pn.z()  };
		return result;
	}

	//����
	Vec3 world2ndc(const Vec& pw, int w, int h) {
		Vec4 pc = mproj * mview * Vec4{ pw };
		Vec3 pn{ pc.x() / pc.w(),pc.y() / pc.w(),pc.z() / pc.w() };
		return pn;
	}
	Vec3 eye,at;

	float Getn()
	{
		return n;
	}

private:
	//��ͼ�任�����ͶӰ����
	Matrix4 mview, mproj;
	Matrix4 fmview, fmproj;
	float n, f;
};


//
// Created by 李曌珩 on 2019-06-23.
//

#ifndef PROGRESSIVE_PHOTON_MAPPING_H
#define PROGRESSIVE_PHOTON_MAPPING_H

#include <vector>
#include "core.h"
#include "scene.h"
#include "json/json.h"
#include "render.h"

class ViewPoint {
	private:
		Vector C,N;
		Color color;
		double strength;
		int x,y;
	public:
		ViewPoint(){}
		ViewPoint(Vector pos,Vector N,Color color,double stgh,int x,int y);
		friend class ProgressivePhotonMapping;
};

class ProgressivePhotonMapping : private Scene, public Render{
	private:
		struct KDTreeNode { 
			public:
				ViewPoint value;
				KDTreeNode* lch;
				KDTreeNode* rch;
				int split_dim;
				Vector bd_max,bd_min;
		};
		template<int dim>
			class ViewPointComparer {
				public:
					bool operator()(const ViewPoint& p1,const ViewPoint& p2) {
						if (dim == 0)
							return p1.C.getX() < p2.C.getX();
						if (dim == 1)
							return p1.C.getY() < p2.C.getY();
						if (dim == 2)
							return p1.C.getZ() < p2.C.getZ();
					}
			};
		KDTreeNode* view_root;

		int max_depth;
		int start_rows;
		int start_cols;
		int bazier_quality;
		int total_round;
		int photon_num;
		double total_brightness;
		double round_decay;
		double initial_r;
		int max_jump;
		std::vector<ViewPoint> view_pts;
		Color* bg_pic;
	public:
		//INIT
		ProgressivePhotonMapping();
		~ProgressivePhotonMapping();
		virtual void accept(const Json::Value& val);

		void run();
		void RayTracing(const Vector& rayO, const Vector& rayD,Color rayC,int xx,int yy,int depth,double lambda,double dist);
		void photonTracing(const Vector& rayO, const Vector& rayD, const Color& rayC,int depth, double r, double lambda, double dist,int diff_count);

		void buildKDTree(KDTreeNode* &now,std::vector<ViewPoint> &lst,int l = -1,int r = -1,int dim = 0);
		void releaseKDTree(KDTreeNode* &node);
		void queryKDTree(KDTreeNode* node, vector<const ViewPoint*> &result, const Vector& pos, double r);
};


#endif

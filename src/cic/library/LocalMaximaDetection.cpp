#include <vector>

std::vector<int> LocMax_pw(std::vector<int> Vi, int min_hight, int min_width)
//std::vector<int> LocMax(std::vector<uchar> Vi, int pP)
{
	
	//Búsqueda de máximos locales

	int Tam=Vi.size();
	std::vector<int>x,y,xProm,yProm, width;
	int j,d1,d2;
	int i=1;
	while (i<Tam)
	{
		j=1;
		if ((Vi[i-1]<Vi[i])&&(Vi[i]>=Vi[i+1]))
		{	
			while ((Vi[i]==Vi[i+j])&&((i+j)<Tam))
			{
				j++;
			}
			if (Vi[i]>Vi[i+j])
			{
				x.push_back(i);
				y.push_back(Vi[i]);
			}
		}
		i+=j;
	}

	int TamM=x.size();
	int pi,fi,px,py,fx,fy,fP, pmin, fmin,P;
	xProm.erase(xProm.begin(),xProm.end());
	for (i=0;i<TamM;i++)
	{
		pi=i-1;
		fi=i+1;
		py=0;
		fy=0;
		fP=0;

		if (pi>=0)
		{	
			while ((pi>=1)&&(y[pi]<=y[i]))//
			{	
				pi--; 
			}
			py=y[pi]>y[i]? y[pi]:0;
		}
		if (fi<TamM)
		{
			while ((fi<TamM-1)&&(y[fi]<=y[i]))//
			{	
				fi++;
			}
			fy=y[fi]>y[i]? y[fi]:0;	
		}
	/////////////////////////////////////////////////////////////
		pmin=Vi[x[i]];
		fmin=Vi[x[i]];
		if ((py==0)&&(fy==0))
		{
			for (j=0;j<Tam;j++)
			{
				if (Vi[j]<pmin)
					pmin=Vi[j];		
//			ROS_INFO("j1: %i",j);
			}
			P=pmin;
			
		}
		else
		{	
			if (pi<0)
			{
				for (j=x[i];j>=0;j--)
				{
					if (Vi[j]<pmin)
						pmin=Vi[j];
				}
				
			}
			else 
			{
				for (j=x[i];j>x[pi];j--)
				{
					if (Vi[j]<pmin)
						pmin=Vi[j];			
				}
				
			}
			if (fi>=TamM-1)
			{
				for (j=x[i];j<Tam;j++)
				{
					if (Vi[j]<fmin)
					fmin=Vi[j];
				}
				
			}
			else 
			{
				for (j=x[i];j<x[fi];j++)
				{
					if (Vi[j]<fmin)
					fmin=Vi[j];
				}
				
			}
			P= pmin>fmin? pmin:fmin;
		}
		j=1;
		while (((x[i]-j)>0)&&(Vi[x[i]-j]-P>(y[i]-P)/2))
			j++;
		d1=j;
		j=1;
		while (((x[i]+j)<Tam)&&(Vi[x[i]+j]-P>(y[i]-P)/2))
			j++;
		d2=j;
		int D=d2+d1;
		if ((y[i]-P>=min_hight)&&(D>=min_width))
		{
			xProm.push_back(x[i]);
			/*ROS_INFO("w: %i",D);
			int prom=y[i]-P;
			ROS_INFO("p: %i",prom);*/
		}

	}

	//return x;
	return xProm;//
}


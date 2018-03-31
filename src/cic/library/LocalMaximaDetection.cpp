#include <vector>

std::vector<int> LocMax_pw(std::vector<int> dataVector, int minProminence, int maxProminence, int minWidth, int maxWidth)
{
	
	//Find local maxima 
	int dataVectorSize=dataVector.size();
	std::vector<int>localMaximaIndex,localMaximaValue,prominenceValue, inRangeLocalMaxima;
	int j;
	int currentValue, nextValue, prevValue;
	int totalLocalMaxima;
	int prevLocalMaximaIndex;
	int nextLocalMaximaIndex;
	int prevHigherLocalMaximaValue;
	int nextHigherLocalMaximaValue;
	int leftWidth, rightWidth, width;
	int prevMinimum, nextMinimum, minimumValue;
	int prominence;
	int i=1;
/********************* FIND LOCAL MAXIMA ***************************/
	while (i<dataVectorSize)
	{
		j=1;
		currentValue=dataVector[i];
		prevValue=dataVector[i-1];
		nextValue=dataVector[i+j];

		if ((prevValue<currentValue)&&(currentValue>=nextValue))
		{	
			while ((currentValue==nextValue)&&((i+j)<dataVectorSize))
			{
				j++;
				nextValue=dataVector[i+j];
			}
			if (currentValue>nextValue)
			{
				localMaximaIndex.push_back(i);
				localMaximaValue.push_back(currentValue);
			}
		}
		i+=j;
	}

	totalLocalMaxima=localMaximaIndex.size();
	prominenceValue.erase(prominenceValue.begin(),prominenceValue.end());

/********************* GET PROMINENCE ***************************/
	for (i=0;i<totalLocalMaxima;i++)
	{
		prevLocalMaximaIndex=i-1;
		nextLocalMaximaIndex=i+1;
		prevHigherLocalMaximaValue=0;
		nextHigherLocalMaximaValue=0;

		if (prevLocalMaximaIndex>=0)
		{	
			while ((prevLocalMaximaIndex>=1)&&(localMaximaValue[prevLocalMaximaIndex]<=localMaximaValue[i]))
			{	
				prevLocalMaximaIndex--; 
			}
			if (localMaximaValue[prevLocalMaximaIndex]>localMaximaValue[i]){
				prevHigherLocalMaximaValue=localMaximaValue[prevLocalMaximaIndex];
			} else {
				prevHigherLocalMaximaValue=0;
			}
		}
		if (nextLocalMaximaIndex<totalLocalMaxima)
		{
			while ((nextLocalMaximaIndex<totalLocalMaxima-1)&&(localMaximaValue[nextLocalMaximaIndex]<=localMaximaValue[i]))//
			{	
				nextLocalMaximaIndex++;
			}
			if (localMaximaValue[nextLocalMaximaIndex]>localMaximaValue[i]){
				nextHigherLocalMaximaValue=localMaximaValue[nextLocalMaximaIndex];
			} else {
				nextHigherLocalMaximaValue=0;
			}
		}
		prevMinimum=dataVector[localMaximaIndex[i]];
		nextMinimum=dataVector[localMaximaIndex[i]];
		minimumValue=dataVector[localMaximaIndex[i]];
		if ((prevHigherLocalMaximaValue==0)&&(nextHigherLocalMaximaValue==0))
		{
			for (j=0;j<dataVectorSize;j++)
			{
				if (dataVector[j]<minimumValue)
					minimumValue=dataVector[j];		
			}
			
			
		}
		else
		{	
			if (prevLocalMaximaIndex<0)
			{
				for (j=localMaximaIndex[i];j>=0;j--)
				{
					if (dataVector[j]<prevMinimum)
						prevMinimum=dataVector[j];
				}
				
			}
			else 
			{
				for (j=localMaximaIndex[i];j>localMaximaValue[prevLocalMaximaIndex];j--)
				{
					if (dataVector[j]<prevMinimum)
						prevMinimum=dataVector[j];			
				}
				
			}
			if (nextLocalMaximaIndex>=totalLocalMaxima-1)
			{
				for (j=localMaximaIndex[i];j<dataVectorSize;j++)
				{
					if (dataVector[j]<nextMinimum)
					nextMinimum=dataVector[j];
				}
				
			}
			else 
			{
				for (j=localMaximaIndex[i];j<localMaximaIndex[nextLocalMaximaIndex];j++)
				{
					if (dataVector[j]<nextMinimum)
					nextMinimum=dataVector[j];
				}
				
			}
			minimumValue= prevMinimum>nextMinimum? prevMinimum:nextMinimum;
		}
		prominence=localMaximaValue[i]-minimumValue;
/*************************** GET WIDTH ********************************/
		j=1;
		while (((localMaximaIndex[i]-j)>0)&&(dataVector[localMaximaIndex[i]-j]-minimumValue>(prominence)/2)) { j++;}
		leftWidth=j;
		j=1;
		while (((localMaximaIndex[i]+j)<dataVectorSize)&&(dataVector[localMaximaIndex[i]+j]-minimumValue>(prominence)/2)) { j++;}
		rightWidth=j;
		int width=leftWidth+rightWidth;

/********************* IS THE PEAK IN RANGE? ***************************/	
		if ((localMaximaValue[i]-minimumValue>=minProminence)&&(localMaximaValue[i]-minimumValue<=maxProminence)&&(width>=minWidth)&&(width<=maxWidth))
		{
			inRangeLocalMaxima.push_back(localMaximaIndex[i]);
		}

	} 
	return inRangeLocalMaxima;
}

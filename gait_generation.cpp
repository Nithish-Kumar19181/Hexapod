#include<math.h>
#include<gait_generation.h>
#include<inverse_kinematics.h> 

// these legAngles are universal legAngles 

void Stand(float height , float legAngles[6][3])
{
    float hexagonSide = 10.5 ;
    // these are the stand coordinates for the hexapod 
    float StandParams[6][3] = {{11.75, 20.351 ,height},
                               {23.50,    0   ,height},
                               {11.75, -20.351,height},
                               {-11.75,-20.351,height},
                               {-23.50, 0     ,height},
                               {-11.75, 20.351,height}
                            };
    for(int i=0; i<6 ; i++)
    {
        for(int j=0 ; j<6 ; j++)
        {
          inverseKinematics(StandParams[i],legAngles[i],5,9,20) ;  
        }
    }
}
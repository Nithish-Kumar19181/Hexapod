# include "inverse_kinematics.h" 
#include<math.h> 
#define pi 3.14159265359 

// This file implements inverse kinematics, translation, and rotation functions
// for a 3D position vector.

// float theta1,theta2,theta3 = inverseKinematics(positionMatrix , L1 , L2 , L3)
// alpha and beta values are clampped between -1 and 1 

// HERE THE ANGLES ARE IN RADIANS CONVERT TO DEGRESS WHEN NEEDED 

bool inverseKinematics(float positionMatrix[] , float jointAngles[3] , float L1 , float L2 , float L3)
{
    float x = positionMatrix[0] ;
    float y = positionMatrix[1] ;
    float z = positionMatrix[2] ;

    float theta1 = atan2(y,x); 
    
    // square of the params 
    float x2 = pow(x,2) ;
    float y2 = pow(y,2) ;
    float z2 = pow(z,2) ;
    float L1_2 = pow(L1,2) ;
    float L2_2 = pow(L2,2) ;
    float L3_2 = pow(L3,2) ;

    float r = sqrt( (x2) + (y2) ) - L1 ;
    float r2 = pow(r,2) ;
    
    float d = sqrt( (r2) + (z2) ) ;
    float d2 = pow(d,2) ;

    if( d > L2+L3 || d < abs(L2-L3))
    {
        Serial.println("Target Position out of reach");
        return false ;
    }

    float phi = atan2(z,r) ;
    // clamping the values of alpha and beta between -1 and 1
    float val_alpha = ((L2_2 + d2 - L3_2) / (2 * L2 * d));
    val_alpha = fmin(fmax(val_alpha, -1.0f), 1.0f);
    float alpha = acos(val_alpha);
    
    float theta2 = -(phi + alpha) ;
    // clamping the values of alpha and beta between -1 and 1
    float val_beta  = (((L2_2) - (d2) + (L3_2)) / (2 * L2 * L3));
    val_beta = fmin(fmax(val_beta, -1.0f), 1.0f);
    float beta = acos(val_beta);
    float theta3 = pi - beta ;

    if (isnan(theta2) || isnan(theta3)) 
    {
        Serial.println("Position unreachable");
        return false;
    }

    jointAngles[0] = theta1 ;
    jointAngles[1] = theta2 ;
    jointAngles[2] = theta3 ;

    return true ;

}

//  function to get the translation of X,Y,Z 
void Translation(float positionMatrix[] , float tran_x , float tran_y , float tran_z)
{
    positionMatrix[0] = positionMatrix[0] + tran_x ;
    positionMatrix[1] = positionMatrix[1] + tran_y ;
    positionMatrix[2] = positionMatrix[2] + tran_z ;
}

//  functions to get the rotation along X,Y,Z 
void RotationZ(float positionMatrix[] , float thetaz)
{
    float rotationz[3][3] = {{cos(thetaz) , -sin(thetaz) ,0},
                             {sin(thetaz) , cos(thetaz) , 0},
                             {0           , 0           , 1}
                            } ;
    float result[3] ;

    for(int i=0 ; i<3 ; i++)
    {
        for(int j=0 ; j<3 ; j++)
        {
            result[i] += rotationz[i][j]*positionMatrix[j] ;
        }
    }
    for (int i = 0; i < 3; i++) 
    {
        positionMatrix[i] = result[i];
    }
}

void RotationX(float positionMatrix[] , float thetax)
{
    float rotationx[3][3] = {{1 , 0           ,  0          },
                             {0 , cos(thetax) , -sin(thetax)},
                             {0 , sin(thetax) ,  cos(thetax)}
                            } ;
    float result[3] ;

    for(int i=0 ; i<3 ; i++)
    {
        for(int j=0 ; j<3 ; j++)
        {
            result[i] += rotationx[i][j]*positionMatrix[j] ;
        }
    }
    for (int i = 0; i < 3; i++) 
    {
        positionMatrix[i] = result[i];
    }
}

void RotationY(float positionMatrix[] ,float thetay)
{
    float rotationy[3][3] = {{cos(thetay) , 0 , sin(thetay)},
                             { 0          , 1 , 0          },
                             {-sin(thetay), 0 , cos(thetay)}
                            };
    float result[3] ;
    
    for(int i=0 ; i<3 ; i++)
    {
        for(int j=0 ; j<3 ; j++)
        {
            result[i] += rotationy[i][j]*positionMatrix[j] ;
        }
    }
    for (int i = 0; i < 3; i++) 
    {
        positionMatrix[i] = result[i];
    }
}
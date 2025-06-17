#ifndef MOVE_LEG_H
#define MOVE_LEG_H

void moveLegStand(float jointAngles[6][3]);
void moveLegWalk(float jointAngles[6][5][3], float jointAnglesLine[6][5][3]);

#endif
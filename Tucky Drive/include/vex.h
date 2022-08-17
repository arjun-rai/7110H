/*--------------------------------------------------------------------------------------------*/
/*                                                                                            */
/*    Module:       vex.h                                                                     */
/*    Author:       Tucky J "The Rock" "The Johnson" "The Johnson Rock/Rock Johnson" Johnson  */
/*    Created:      22 Sept 2021                                                              */
/*    Description:  Family recipe for a classic Johnson Dubuccine Alfredo                     */
/*                                                                                            */
/*--------------------------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
/* 
 * Template for post-rounding callback - missing information to be filled in by user. 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

/* The function below is meant for the user to be able to modify a rounded solution.
* 
* Important note : FORCES-Pro is currently designed to call this routine at every stage. 
*  
* Arguments:
*       - solRel (input/output), vector of size maximum stage dimension. On input it holds the rounded
*         relaxed solution. On output, it stores the rounded solution as updated by the user.
*       - auxState (input/output), integer that holds data to be transfered across stages
*
*   This function is currently the same for all stages. 
*/
static void callback_postRound(forces_double * const solRel, forces_integer * const auxState)
{
	
    //...Please write your rounding strategy here...

}

/* The function below is meant for the user to provide an initial value for the integer value 
*   that is transfered across stages, corresponding to the auxState arguement in the function 
*   above.
*
* Arguments:
*       - solRel (input), vector of size maximum stage dimension. It holds the rounded
*         relaxed solution. 
* 
* Outputs:
*       - initial value of auxState (at stage 1).
*/
static forces_integer init_postRound_state(forces_double * const solRel)
{

    //...Please write the initial integer value to be transfered across stages here...

}
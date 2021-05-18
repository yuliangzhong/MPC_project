/* 
 * Template for post-branching callback - missing information to be filled in by user. 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

/* The function below is meant for the user to be able to update the integer lower and upper bounds after branching.
* 
* Important note : FORCES-Pro is currently designed to call this routine on the left and right sons of a branched node. 
*  
* Arguments:
*       - branch_stage (input), index of stage to which branched variable belongs.
*		- branch_id (input), index of stage variable that has been branched.
*		- lowBnd (input/output), array that contains integer lower bounds after branching. 
*					Its dimension is equal to the total number of integers across all stages.
*		- uppBnd (input/output), array that contains integer upper bounds after branching.
*					Its dimension is equal to the total number of integers across all stages.
*		- parentSolRel (input), array that contains the parent solution of relaxation.
*					Its dimension is equal to the primal variable dimension across all stages. 
*
*	You are expected to give the callback name after the body and @ and remove the <>. 
*
*/
static forces_integer <Name of user callback>(const forces_integer branch_stage, const forces_integer branch_id, forces_integer * const lowBnd, forces_integer * const uppBnd, const forces_double * const parentSolRel)
{
	
	//...Please write your routine for updating the node integer bounds here...
	//...Return 0 if something wrong happened...

	return( 1 );
}@<Name of user callback>
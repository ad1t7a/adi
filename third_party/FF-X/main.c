

/*
 * THIS SOURCE CODE IS SUPPLIED  ``AS IS'' WITHOUT WARRANTY OF ANY KIND, 
 * AND ITS AUTHOR AND THE JOURNAL OF ARTIFICIAL INTELLIGENCE RESEARCH 
 * (JAIR) AND JAIR'S PUBLISHERS AND DISTRIBUTORS, DISCLAIM ANY AND ALL 
 * WARRANTIES, INCLUDING BUT NOT LIMITED TO ANY IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND
 * ANY WARRANTIES OR NON INFRINGEMENT.  THE USER ASSUMES ALL LIABILITY AND
 * RESPONSIBILITY FOR USE OF THIS SOURCE CODE, AND NEITHER THE AUTHOR NOR
 * JAIR, NOR JAIR'S PUBLISHERS AND DISTRIBUTORS, WILL BE LIABLE FOR 
 * DAMAGES OF ANY KIND RESULTING FROM ITS USE.  Without limiting the 
 * generality of the foregoing, neither the author, nor JAIR, nor JAIR's
 * publishers and distributors, warrant that the Source Code will be 
 * error-free, will operate without interruption, or will meet the needs 
 * of the user.
 */






/*********************************************************************
 * File: main.c
 * Description: The main routine for the FastForward Planner.
 *
 * Author: Joerg Hoffmann 2000
 * 
 *********************************************************************/ 








#include "ff.h"

#include "memory.h"
#include "output.h"

#include "parse.h"

#include "inst_pre.h"
#include "inst_easy.h"
#include "inst_hard.h"
#include "inst_final.h"

#include "orderings.h"

#include "relax.h"
#include "search.h"











/*
 *  ----------------------------- GLOBAL VARIABLES ----------------------------
 */












/*******************
 * GENERAL HELPERS *
 *******************/








/* used to time the different stages of the planner
 */
float gtempl_time = 0, greach_time = 0, grelev_time = 0, gconn_time = 0;
float gsearch_time = 0;


/* the command line inputs
 */
struct _command_line gcmd_line;

/* number of states that got heuristically evaluated
 */
int gevaluated_states = 0;

/* maximal depth of breadth first search
 */
int gmax_search_depth = 0;





/***********
 * PARSING *
 ***********/







/* used for pddl parsing, flex only allows global variables
 */
int gbracket_count;
char *gproblem_name;

/* The current input line number
 */
int lineno = 1;

/* The current input filename
 */
char *gact_filename;

/* The pddl domain name
 */
char *gdomain_name = NULL;

/* loaded, uninstantiated operators
 */
PlOperator *gloaded_ops = NULL;

/* stores initials as fact_list 
 */
PlNode *gorig_initial_facts = NULL;

/* not yet preprocessed goal facts
 */
PlNode *gorig_goal_facts = NULL;

/* axioms as in UCPOP before being changed to ops
 */
PlOperator *gloaded_axioms = NULL;

/* the types, as defined in the domain file
 */
TypedList *gparse_types = NULL;

/* the constants, as defined in domain file
 */
TypedList *gparse_constants = NULL;

/* the predicates and their arg types, as defined in the domain file
 */
TypedListList *gparse_predicates = NULL;

/* the objects, declared in the problem file
 */
TypedList *gparse_objects = NULL;


/* connection to instantiation ( except ops, goal, initial )
 */

/* all typed objects 
 */
FactList *gorig_constant_list = NULL;

/* the predicates and their types
 */
FactList *gpredicates_and_types = NULL;












/*****************
 * INSTANTIATING *
 *****************/









/* global arrays of constant names,
 *               type names (with their constants),
 *               predicate names,
 *               predicate aritys,
 *               defined types of predicate args
 */
Token gconstants[MAX_CONSTANTS];
int gnum_constants = 0;
Token gtype_names[MAX_TYPES];
int gtype_consts[MAX_TYPES][MAX_TYPE];
Bool gis_member[MAX_CONSTANTS][MAX_TYPES];
int gtype_size[MAX_TYPES];
int gnum_types = 0;
Token gpredicates[MAX_PREDICATES];
int garity[MAX_PREDICATES];
int gpredicates_args_type[MAX_PREDICATES][MAX_ARITY];
int gnum_predicates = 0;
Bool gderived[MAX_PREDICATES];





/* the domain in integer (Fact) representation
 */
Operator_pointer goperators[MAX_OPERATORS];
int gnum_operators = 0;
Fact *gfull_initial;
int gnum_full_initial = 0;
WffNode *ggoal = NULL;




/* stores inertia - information: is any occurence of the predicate
 * added / deleted in the uninstantiated ops ?
 */
Bool gis_added[MAX_PREDICATES];
Bool gis_deleted[MAX_PREDICATES];



/* splitted initial state:
 * initial non static facts,
 * initial static facts, divided into predicates
 * (will be two dimensional array, allocated directly before need)
 */
Facts *ginitial = NULL;
int gnum_initial = 0;
Fact **ginitial_predicate;
int *gnum_initial_predicate;



/* the type numbers corresponding to any unary inertia
 */
int gtype_to_predicate[MAX_PREDICATES];
int gpredicate_to_type[MAX_TYPES];

/* (ordered) numbers of types that new type is intersection of
 */
TypeArray gintersected_types[MAX_TYPES];
int gnum_intersected_types[MAX_TYPES];



/* splitted domain: hard n easy ops
 */
Operator_pointer *ghard_operators;
int gnum_hard_operators;
NormOperator_pointer *geasy_operators;
int gnum_easy_operators;



/* so called Templates for easy ops: possible inertia constrained
 * instantiation constants
 */
EasyTemplate *geasy_templates;
int gnum_easy_templates;



/* first step for hard ops: create mixed operators, with conjunctive
 * precondition and arbitrary effects
 */
MixedOperator *ghard_mixed_operators;
int gnum_hard_mixed_operators;



/* hard ''templates'' : pseudo actions
 */
PseudoAction_pointer *ghard_templates;
int gnum_hard_templates;



/* store the final "relevant facts"
 */
Fact grelevant_facts[MAX_RELEVANT_FACTS];
int gnum_relevant_facts = 0;
int gnum_pp_facts = 0;



/* the final actions and problem representation
 */
Action *gactions;
int gnum_actions;
State ginitial_state;
State ggoal_state;









/**********************
 * CONNECTIVITY GRAPH *
 **********************/







/* one ops (actions) array ...
 */
OpConn *gop_conn;
int gnum_op_conn;



/* one effects array ...
 */
EfConn *gef_conn;
int gnum_ef_conn;



/* one facts array.
 */
FtConn *gft_conn;
int gnum_ft_conn;









/*******************
 * SEARCHING NEEDS *
 *******************/








/* the goal state, divided into subsets
 */
State *ggoal_agenda;
int gnum_goal_agenda;



/* byproduct of fixpoint: applicable actions
 */
int *gnaxA;
int gnum_naxA;
int *gaxA;
int gnum_axA;



/* communication from extract 1.P. to search engines:
 * 1P action choice
 */
int *gH;
int gnum_H;



/* the effects that are considered true in relaxed plan
 */
int *gin_plan_E;
int gnum_in_plan_E;



/* always stores (current) serial plan
 */
int gplan_ops[MAX_PLAN_LENGTH];
int gnum_plan_ops = 0;



/* stores the states that the current plan goes through
 * ( for knowing where new agenda entry starts from )
 */
State gplan_states[MAX_PLAN_LENGTH + 1];



int *gaxdels, gnum_axdels;

int gnum_strata;




/*
 *  ----------------------------- HEADERS FOR PARSING ----------------------------
 * ( fns defined in the scan-* files )
 */







void get_fct_file_name( char *filename );
void load_ops_file( char *filename );
void load_fct_file( char *filename );











/*
 *  ----------------------------- MAIN ROUTINE ----------------------------
 */




struct tms lstart, lend;







/*
 *  ----------------------------- HELPING FUNCTIONS ----------------------------
 */










FILE *out;


void output_planner_info( void )

{

  char name[MAX_LENGTH];

  printf( "\n\ntime spent: %7.2f seconds instantiating %d easy, %d hard action templates", 
	  gtempl_time, gnum_easy_templates, gnum_hard_mixed_operators );
  printf( "\n            %7.2f seconds reachability analysis, yielding %d facts and %d actions", 
	  greach_time, gnum_pp_facts, gnum_actions );
  printf( "\n            %7.2f seconds creating final representation with %d relevant facts", 
	  grelev_time, gnum_relevant_facts );
  printf( "\n            %7.2f seconds building connectivity graph",
	  gconn_time );
  printf( "\n            %7.2f seconds searching, evaluating %d states, to a max depth of %d", 
	  gsearch_time, gevaluated_states, gmax_search_depth );
  printf( "\n            %7.2f seconds total time", 
	  gtempl_time + greach_time + grelev_time + gconn_time + gsearch_time );

  printf("\n\n");

  sprintf( name, "TIME-%s", gcmd_line.fct_file_name );
  if ( (out = fopen( name, "w")) == NULL ) {
    printf("\n\nCan't open output file!\n\n");
    return;
  }
  times( &lend );
  fprintf(out, "%.2f\n", 
	 ((float) (lend.tms_utime - lstart.tms_utime + lend.tms_stime - lstart.tms_stime)) / 100.0);
  fclose( out );

  sprintf( name, "SEARCHTIME-%s", gcmd_line.fct_file_name );
  if ( (out = fopen( name, "w")) == NULL ) {
    printf("\n\nCan't open output file!\n\n");
    return;
  }
  times( &lend );
  fprintf(out, "%.2f\n", gsearch_time);
  fclose( out );

  exit( 0 );

}



void print_official_result( void )

{

  char name[MAX_LENGTH];

  sprintf( name, "TIME-%s", gcmd_line.fct_file_name );

  if ( (out = fopen( name, "w")) == NULL ) {
    printf("\n\nCan't open output file!\n\n");
    return;
  }

  times( &lend );
  fprintf(out, "%.2f\n", 
	 ((float) (lend.tms_utime - lstart.tms_utime + lend.tms_stime - lstart.tms_stime)) / 100.0);

  fclose( out );

}



void print_official_op_name( int index )

{

  int i;
  Action *a = gop_conn[index].action;

  if ( a->norm_operator ||
       a->pseudo_action ) {
    fprintf(out, "(%s", a->name ); 
    for ( i = 0; i < a->num_name_vars; i++ ) {
      fprintf(out, " %s", gconstants[a->name_inst_table[i]]);
    }
    fprintf(out, ")");
  }

}



void ff_usage( void )

{

  printf("\nusage of ff:\n");

  printf("\nOPTIONS   DESCRIPTIONS\n\n");
  printf("-p <str>    path for operator and fact file\n");
  printf("-o <str>    operator file name\n");
  printf("-f <str>    fact file name\n\n");
  printf("-i <num>    run-time information level( preset: 1 )\n");
  printf("      0     only times\n");
  printf("      1     problem name, planning process infos\n");
  printf("    101     parsed problem data\n");
  printf("    102     cleaned up ADL problem\n");
  printf("    103     collected string tables\n");
  printf("    104     encoded domain\n");
  printf("    105     predicates inertia info\n");
  printf("    106     splitted initial state\n");
  printf("    107     domain with Wff s normalized\n");
  printf("    108     domain with NOT conds translated\n");
  printf("    109     splitted domain\n");
  printf("    110     cleaned up easy domain\n");
  printf("    111     unaries encoded easy domain\n");
  printf("    112     effects multiplied easy domain\n");
  printf("    113     inertia removed easy domain\n");
  printf("    114     easy action templates\n");
  printf("    115     cleaned up hard domain representation\n");
  printf("    116     mixed hard domain representation\n");
  printf("    117     final hard domain representation\n");
  printf("    118     reachability analysis results\n");
  printf("    119     facts selected as relevant\n");
  printf("    120     final domain and problem representations\n");
  printf("    121     connectivity graph\n");
  printf("    122     fixpoint result on each evaluated state\n");
  printf("    123     1P extracted on each evaluated state\n");
  printf("    124     H set collected for each evaluated state\n");
  printf("    125     False sets of goals <GAM>\n");
  printf("    126     detected ordering constraints leq_h <GAM>\n");
  printf("    127     the Goal Agenda <GAM>\n");



/*   printf("    109     reachability analysis results\n"); */
/*   printf("    110     final domain representation\n"); */
/*   printf("    111     connectivity graph\n"); */
/*   printf("    112     False sets of goals <GAM>\n"); */
/*   printf("    113     detected ordering constraints leq_h <GAM>\n"); */
/*   printf("    114     the Goal Agenda <GAM>\n"); */
/*   printf("    115     fixpoint result on each evaluated state <1Ph>\n"); */
/*   printf("    116     1P extracted on each evaluated state <1Ph>\n"); */
/*   printf("    117     H set collected for each evaluated state <1Ph>\n"); */
  
  printf("\n-d <num>    switch on debugging\n\n");

}



Bool process_command_line( int argc, char *argv[] )

{

  char option;

  gcmd_line.display_info = 1;
  gcmd_line.debug = 0;
  
  memset(gcmd_line.ops_file_name, 0, MAX_LENGTH);
  memset(gcmd_line.fct_file_name, 0, MAX_LENGTH);
  memset(gcmd_line.path, 0, MAX_LENGTH);

  while ( --argc && ++argv ) {
    if ( *argv[0] != '-' || strlen(*argv) != 2 ) {
      return FALSE;
    }
    option = *++argv[0];
    switch ( option ) {
    default:
      if ( --argc && ++argv ) {
	switch ( option ) {
	case 'p':
	  strncpy( gcmd_line.path, *argv, MAX_LENGTH );
	  break;
	case 'o':
	  strncpy( gcmd_line.ops_file_name, *argv, MAX_LENGTH );
	  break;
	case 'f':
	  strncpy( gcmd_line.fct_file_name, *argv, MAX_LENGTH );
	  break;
	case 'i':
	  sscanf( *argv, "%d", &gcmd_line.display_info );
	  break;
	case 'd':
	  sscanf( *argv, "%d", &gcmd_line.debug );
	  break;
	default:
	  printf( "\nff: unknown option: %c entered\n\n", option );
	  return FALSE;
	}
      } else {
	return FALSE;
      }
    }
  }

  return TRUE;

}


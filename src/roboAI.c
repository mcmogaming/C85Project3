/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Version: 1.1 - Updated Jun 30, 2018 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!

double dottie(double vx, double vy, double ux, double uy)
{
 // Returns the dot product of the two vectors [vx,vy] and [ux,uy]
 return (vx*ux)+(vy*uy);
}

double crossie_sign(double vx, double vy, double ux, double uy)
{
 // Returns the sign of the Z component of the cross product of 
 //   vectors [vx vy 0] and [ux uy 0]
 // MIND THE ORDER! v rotating onto u.     
 // i   j   k 
 // vx  vy  0
 // ux  uy  0
    
 if ((vx*uy)-(ux*vy)<0) return -1;
 else return 1;
}

void clear_motion_flags(struct RoboAI *ai)
{
 // Reset all motion flags. See roboAI.h for what each flag represents
 // *You may or may not want to use these*
 ai->st.mv_fwd=0; 
 ai->st.mv_back=0;
 ai->st.mv_bl=0;
 ai->st.mv_br=0;
 ai->st.mv_fl=0;
 ai->st.mv_fr=0;
}

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Red
 //                   1 -> Yellow or Green (see below)
 //                   2 -> Blue
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.65;                                              // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // For whatever reason, green printed paper tends to be problematic, so, we will use yellow instead,
 // but, the code below can be made to detect a green uniform if needed by commenting/uncommenting
 // the appropriate line just below.
 // The reference colour here is in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  if (col==0) {vr_x=cos(0); vr_y=sin(0);}   // RED                                
  else if (col==2) {vr_x=cos(2.0*PI*(60.0/360.0));
                    vr_y=sin(2.0*PI*(60.0/360.0));}  // YELLOW
  else if (col==1) {vr_x=cos(2.0*PI*(240.0/360.0));
                   vr_y=sin(2.0*PI*(240.0/360.0));}      // BLUE

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {

  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}


void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Heading (a unit vector in the direction of motion). Not valid
 //   while rotating - possibly valid while turning
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // Note that the blob data
 // structure itself contains another useful vector with the blob
 // orientation (obtained directly from the blob shape, valid at all
 // times even under rotation, but can be pointing backward!)
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 double NOISE_VAR=5;

 // Reset ID flags
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;

 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute heading direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob

  // Adjust Y position if we have calibration data
  if (fabs(p->adj_Y[0][0])>.1)
  {
   dmax=384.0-p->adj_Y[0][0];
   dmin=767.0-p->adj_Y[1][0];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][0]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;

  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }

  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
 }
 else ai->st.self=NULL;

 // ID our opponent
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,0);
 else p=id_coloured_blob2(ai,blobs,1);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	

  if (fabs(p->adj_Y[0][1])>.1)
  {
   dmax=384.0-p->adj_Y[0][1];
   dmin=767.0-p->adj_Y[1][1];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][1]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/5.0;

 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30);			// Start forward motion to establish heading
					// Will move for a few frames.

 track_agents(ai,blobs);		// Call the tracking function to find each agent

 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 clear_motion_flags(ai);
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 track_agents(ai,blobs);
}

void PD_align(BotInfo myBot, double ux, double uy, double Kp, double Kd, double minPower)
{
    static double err_old=0;
    double err_new;
    double derr;
    double sgn;
    double PD;

//    sgn=crossie_sign(myBot.Heading[0],myBot.Heading[1],ux,uy);
    sgn=crossie_sign(ux,uy,myBot.Heading[0],myBot.Heading[1]);
    err_new=acos(dottie(myBot.Heading[0],myBot.Heading[1],ux,uy));
    
    derr=err_new-err_old;
    PD=sgn*(Kp*err_new) + (Kd*(derr/PI));
    err_old=err_new;

//    printf("PID(): err_new=%lf, sgn=%lf, dErr=%lf, dptf=%lf, P=%lf, D=%lf, PD=%lf\n",err_new,sgn,derr,myBot.dtpf,Kp*err_new, Kd*(derr/PI), PD);

    PD=sgn*15;
    
    if (fabs(PD)>100) PD=sgn*100.0;
    if (fabs(PD)<minPower) PD=sgn*minPower;
    
    BT_motor_port_start(RIGHT_MOTOR, (char)roundl(PD));
    BT_motor_port_start(LEFT_MOTOR, (char)roundl(-PD));
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is the main AI loop.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your NXT to get damaged!

  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static BotInfo myBot;
  static BotInfo theirBot;
  static BotInfo fluffy;
  static double ux,uy,len;
  double angDif;
  static int count=0;
  static double old_dx=0, old_dy=0;
  
  // change to the ports representing the left and right motors
  char lport=MOTOR_A;
  char rport=MOTOR_B;
 
  if (ai->st.state==1||ai->st.state==2||ai->st.state==101||ai->st.state==102||ai->st.state==201||ai->st.state==202)
  {
     if (ai->st.self!=NULL)
     {
      // Update dtpf and robot heading
      angDif=dottie(ai->st.self->dx,ai->st.self->dy,old_dx,old_dy);
      if (angDif<0)
      {
       myBot.Heading[0]=-ai->st.self->dx;
       myBot.Heading[1]=-ai->st.self->dy;
      }
      else
      {
       myBot.Heading[0]=ai->st.self->dx;
       myBot.Heading[1]=ai->st.self->dy;
      }
      myBot.dtpf=acos(angDif);       // in Radians!
      angDif=dottie(ai->st.self->dx,ai->st.self->dy,old_dx,old_dy);
      if (fabs(angDif)<.8) 
      {
          myBot.Heading[0]=old_dx;
          myBot.Heading[1]=old_dy;
      }
          
      // Match direction
      if (ai->st.state==1)
      {         
          fprintf(stderr,"Please enter desired direction ux:\n");
          fscanf(stdin,"%lf",&ux);
          getchar();
          fprintf(stderr,"Please enter desired direction uy:\n");
          fscanf(stdin,"%lf",&uy);
          getchar();
          len=sqrt((ux*ux)+(uy*uy));
          ux=ux/len;
          uy=uy/len;
          fprintf(stderr,"Desired direction vector is [%lf, %lf], heading is [%lf, %lf]\n",ux,uy,myBot.Heading[0],myBot.Heading[1]);
      }
      angDif=dottie(ux,uy,myBot.Heading[0],myBot.Heading[1]);
      printf("Current parameters: blob=[%lf %lf], heading=[%lf, %lf], old heading=[%lf %lf], dot_blobHead=%lf, dot_newOld=%lf, angBlob=%lf, angHead=%lf\n",ai->st.self->dx, ai->st.self->dy,myBot.Heading[0],myBot.Heading[1],old_dx,old_dy,dottie(ai->st.self->dx,ai->st.self->dy,myBot.Heading[0],myBot.Heading[1]),dottie(myBot.Heading[0],myBot.Heading[1],old_dx,old_dy),atan2(ai->st.self->dy,ai->st.self->dx),atan2(myBot.Heading[1],myBot.Heading[0]));
//      fprintf(stderr,"Cosine of angle difference is: %lf\n",angDif);
      angDif=acos(angDif);
//      fprintf(stderr,"Actual angular difference: %lf\n",angDif);
      count++;
      if (count>150)
      {
          ai->st.state=500;
          BT_all_stop(0);
          count=0;
      }
     
      if ((ai->st.state==1||ai->st.state==101||ai->st.state==201)&&angDif>ANGLE_DIFF_THRESH)
          ai->st.state++;                        // Not aligned - go to state **2
             
      if (ai->st.state==2||ai->st.state==102||ai->st.state==202)
      {
         if (angDif>ANGLE_DIFF_THRESH)
         {
//             fprintf(stderr,"Angle difference is %lf\n",angDif);
//             fprintf(stderr,"Executing PID alignment...\n");
             PD_align(myBot,ux,uy,KD,KP,15);
         }
         else
         {
             // Aligned - go back to state **1
//             fprintf(stderr,"Alignment achieved! - going back to request a new vector\n");
             ai->st.state--;
             BT_all_stop(0);
             clear_motion_flags(ai);
         }
      }
      old_dx=myBot.Heading[0];
      old_dy=myBot.Heading[1];
     }		// End if (ai->st.self!=NULL)
  }
 

 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  memset(&myBot,0,sizeof(BotInfo));
  memset(&theirBot,0,sizeof(BotInfo));
  memset(&fluffy,0,sizeof(BotInfo));
  myBot.Position[0]=-1;
  myBot.Position[1]=-1;
  theirBot.Position[0]=-1;              // Set to -1 if not identified and being tracked
  theirBot.Position[1]=-1;
  fluffy.Position[0]=-1;
  fluffy.Position[1]=-1;
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	// The id_bot() routine will change the AI state to initial state + 1
  {				// if robot identification is successful.
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;
   BT_all_stop(0);
   clear_motion_flags(ai);
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.self->mx,ai->st.self->my,ai->st.self->dx,ai->st.self->dy,ai->st.state);
   
   if (ai->st.self!=NULL)
   {
       myBot.Motion[0]=ai->st.self->mx;
       myBot.Motion[1]=ai->st.self->my;
       myBot.Velocity[0]=ai->st.self->vx;
       myBot.Velocity[1]=ai->st.self->vy;
       if (((ai->st.self->mx*ai->st.self->dx)+(ai->st.self->my*ai->st.self->dy))<0)
       {
           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
       }
       myBot.Heading[0]=ai->st.self->dx;
       myBot.Heading[1]=ai->st.self->dy;
       myBot.Position[0]=ai->st.self->cx;
       myBot.Position[1]=ai->st.self->cy;
       myBot.bel=.9; 
       myBot.dtpf=0;
       old_dx=ai->st.self->dx;
       old_dy=ai->st.self->dy;
   }
  
   if (ai->st.opp!=NULL)
   {
       theirBot.Motion[0]=ai->st.opp->mx;
       theirBot.Motion[1]=ai->st.opp->my;
       theirBot.Velocity[0]=ai->st.opp->vx;
       theirBot.Velocity[1]=ai->st.opp->vy;
       if (((ai->st.opp->mx*ai->st.opp->dx)+(ai->st.opp->my*ai->st.opp->dy))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
       }       
       theirBot.Heading[0]=ai->st.opp->dx;
       theirBot.Heading[1]=ai->st.opp->dy;
       theirBot.Position[0]=ai->st.opp->cx;
       theirBot.Position[1]=ai->st.opp->cy;
       theirBot.bel=.9; 
       theirBot.dtpf=0;
   }
   
   if (ai->st.ball!=NULL)
   {
       fluffy.Motion[0]=ai->st.ball->mx;
       fluffy.Motion[1]=ai->st.ball->my;
       fluffy.Velocity[0]=ai->st.ball->vx;
       fluffy.Velocity[1]=ai->st.ball->vy;
       fluffy.Heading[0]=ai->st.ball->dx;
       fluffy.Heading[1]=ai->st.ball->dy;
       fluffy.Position[0]=ai->st.ball->cx;
       fluffy.Position[1]=ai->st.ball->cy;
       fluffy.bel=.9; 
       fluffy.dtpf=0;
   }

  }
  
  // Initialize BotInfo structures
   
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
  fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
  track_agents(ai,blobs);		// Currently, does nothing but endlessly track
 }

}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/



///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Neuron
//  Workfile:        HHNeuron.cpp
//  Revision:        1.0 - 21 August, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Hodgkin-Huxley neural model definition
//
///////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "HHNeuron.h"

namespace Neuron
{
  LIBRARY_API HHNeuron::HHNeuron () :
    output_(0.0)
  {
  }


  LIBRARY_API HHNeuron::~HHNeuron ()
  {
  }


  LIBRARY_API void funcp (double t, double* y, double* f, double* p)
  {
    double g_NA_bar = p[0];
    double g_K1_bar = p[1];
    double g_leak_bar = p[2];
    double E_NA = p[3];
    double E_K = p[4];
    double E_L = p[5];

    double sum;
    double I_NA,I_K1;

    ///////////////////////////////////////////////////////////////////////////
    //                                  Units                                //
    ///////////////////////////////////////////////////////////////////////////
    // Methods in Neuronal Modeling: Chapter 4: Dynamic Channels             //
    //  by Yamada, Koch and Adams                                            //
    ///////////////////////////////////////////////////////////////////////////
    //  voltage       mV
    //  current       na
    //  time          ms
    //  concentration mM
    //  conductance   micro Siemens
    //  capacitance   nF
    ///////////////////////////////////////////////////////////////////////////
    //                           Y vector assignments                        //
    ///////////////////////////////////////////////////////////////////////////
    //  y[0] = V
    //  y[1] = m_NA
    //  y[2] = h_NA
    //  y[3] = m_K1
    //  y[4] = h_K1
    ///////////////////////////////////////////////////////////////////////////

    //! Fast sodium current:
    //! Activation parameter for I_NA
    double alpha_mNA;
    double beta_mNA;
    double m_NA_infinity;
    double t_m_NA;

    sum = y[0] + 33.0;
    if (sum > 0.0)
    {
      alpha_mNA = 0.36 * sum/(1.0 - exp (-sum/3.0));
    }
    else
    {
      alpha_mNA = 0.36 * exp (sum/ 3.0) * sum/(exp (sum/3.0) - 1.0);
    }

    sum = y[0] + 42.0;
    if (sum > 0.0)
    {
      beta_mNA = -0.4 * exp (-sum/20.0)*sum/(exp( -sum/20.0) - 1.0);
    }
    else
    {
      beta_mNA = -0.4 * sum/(1.0 - exp (sum/20.0));
    }

    m_NA_infinity = alpha_mNA/(alpha_mNA+beta_mNA);
    t_m_NA = 2.0/(alpha_mNA+beta_mNA);
    f[1] = (m_NA_infinity - y[1])/t_m_NA;

    //! Inactivation parameter for I_NA
    double alpha_hNA;
    double beta_hNA;
    double h_NA_infinity;
    double t_h_NA;

    sum = y[0]+55.0;
    if (sum < 0.0)
    {
      alpha_hNA = -0.1*sum/(1.0 - exp(sum/ 6.0));
    }
    else
    {
      alpha_hNA = -0.1*exp( -sum/ 6.0)*sum/(exp( -sum/ 6.0) - 1.0);
    }

    if (y[0] > 0.0)
    {
      beta_hNA = 4.5/(1.0 + exp(-y[0]/10.0));
    }
    else
    {
      beta_hNA = 4.5*exp(y[0]/10.0)/(exp(y[0]/10.0) + 1.0);
    }

    h_NA_infinity = alpha_hNA/(alpha_hNA+beta_hNA);
    t_h_NA        = 2.0/(alpha_hNA+beta_hNA);
    f[2]          = (h_NA_infinity - y[2])/t_h_NA; 

    I_NA = g_NA_bar*(y[0]-E_NA)*y[1]*y[1]*y[2]; 
  
    //! Transient, outward potassium current
    //! Activation parameter for I_K
    double t_m_K1;
    double m_K1_infinity;
 
    sum = y[0]+42.0;
    if(sum<0)
    {
      m_K1_infinity = exp(sum/13.0)/(exp(sum/13.0)+1.0);
    }
    else
    {
      m_K1_infinity = 1.0/(1.0+exp(-sum/13.0));
    }

    t_m_K1 = 1.38;
    f[3] = (m_K1_infinity - y[3])/t_m_K1;

    // inactivation parameter for I_K
    double h_K1_infinity;
    double t_h_K1;

    sum = y[0]+110.0;
    if (sum < 0.0)
    {
      h_K1_infinity = 1.0/(1.0+exp( sum/18.0));
    }
    else
    {
      h_K1_infinity = exp( -sum/18.0)/(exp(-sum/18.0)+1.0);
    }

    if (y[0] < -80.0)
    {
      t_h_K1 = 50.0;
    }
    else
    {
      t_h_K1 = 150.0;
    }

    f[4] = (h_K1_infinity - y[4])/t_h_K1; 

    I_K1 = g_K1_bar*(y[0]-E_K)*y[3]*y[4];

    //! Leakage current
    double I_leak = g_leak_bar*(y[0]-E_L);

    //! Cell voltage equation
    static double C_N = .15;
    double external_current = 0.0;

    static double delta_t = 1.0;
    static double t_start = 10.0;
    static double t_end = t_start + delta_t;
    static double injection = 20.0;

    if (t >= t_start && t <= t_end)
    {
      external_current = injection; 
    }
   
    f[0] = (external_current- I_NA - I_K1 - I_leak) / C_N;
  }


  double rest (double E_M, double* p, double* q)
  {
    double sum;

    double g_NA_bar    = p[0];
    double g_K1_bar    = p[1];
    double g_leak_bar  = p[2];
    double E_NA        = p[3];
    double E_K         = p[4];
    //double E_L         = p[5];

//    cout << "In rest calculation: parameters are p = " << p << endl;

    ///////////////////////////////////////////////////////////////////////////
    //                     How do we choose E_L and g_L?                     //
    ///////////////////////////////////////////////////////////////////////////
    //  At equilibrium we must have:
    //    g_total = g_NA_bar+g_K1_bar;
    //    ratio_NA = g_NA_bar/(g_total+g_leak_bar);
    //    ratio_K1 = g_K1_bar/(g_total+g_leak_bar);
    //    ratio_L = g_leak_bar/(g_total+g_leak_bar);
    //    E_M = ratio_NA*E_NA + ratio_K1*E_K + ratio_L*E_L;
    //
    //  For E_M we must also have:
    //    I_NA = g_NA_bar*(E_M-E_NA)*m_NA_infinity*m_NA_infinity*h_NA_infinity;    
    //    I_K1 = g_K1_bar*(E_M-E_K)*m_K1_infinity*h_K1_infinity;
    //    sum = I_NA + I_K1; 
    //    g_leak_bar  = -sum/(E_M-E_L);
    //
    //  So:
    //    E_M*(g_total+g_leak_bar) = g_K*E_NA + g_K1*E_K1 + g_leak_bar*E_L;
    //
    //  Let:
    //    S1 = g_K*E_NA + g_K1*E_K1 ;
    //    S2 = sum;
    //
    //  Then:
    //    g_leak_bar = -S2/(E_M-E_L);
    //
    //  yielding:
    //    E_M*(g_total -S2/(E_M-E_L)) = S1 + (-S2/(E_M-E_L)*E_L;
    ///////////////////////////////////////////////////////////////////////////
  
    //! Activation parameter for I_NA
    double alpha_mNA;
    double beta_mNA;
    double m_NA_infinity;
  
    sum = E_M+33.0;
    if(sum>0)
    {
      alpha_mNA = 0.36*sum/(1.0 - exp(-sum/3.0));
    }
    else
    {
      alpha_mNA = 0.36*exp( sum/ 3.0)*sum/(exp(sum/ 3.0) - 1.0);
    }
    sum = E_M+42.0;
    if (sum > 0.0)
    {
      beta_mNA = -0.4*exp( -sum/20.0)*sum/(exp( -sum/20.0) - 1.0);
    }
    else
    {
      beta_mNA = -0.4*sum/(1.0 - exp(sum/20.0));
    }
    m_NA_infinity = alpha_mNA/(alpha_mNA+beta_mNA);

    //! Inactivation parameter for I_NA
    double alpha_hNA;
    double beta_hNA;
    double h_NA_infinity;

    sum = E_M+55.0;
    if (sum < 0.0)
    {
      alpha_hNA = -0.1*sum/(1.0 - exp(sum/ 6.0));
    }
    else
    {
      alpha_hNA = -0.1*exp( -sum/ 6.0)*sum/(exp( -sum/ 6.0) - 1.0);
    }

    if (E_M > 0.0)
    {
      beta_hNA = 4.5/(1.0 + exp(-E_M/10.0));
    }
    else
    {
      beta_hNA = 4.5*exp(E_M/10.0)/(exp(E_M/10.0) + 1.0);
    }
    h_NA_infinity = alpha_hNA/(alpha_hNA+beta_hNA);

    double I_NA = g_NA_bar*(E_M-E_NA)*m_NA_infinity*m_NA_infinity*h_NA_infinity; 

    //! Transient, Outward Potassium Current
    // activation parameter for I_K
//    double t_m_K1;
    double m_K1_infinity;

    sum = E_M+42.0;
    if(sum<0)
    {
      m_K1_infinity = exp(sum/13.0)/(exp(sum/13.0)+1.0);
    }
    else
    {
      m_K1_infinity = 1.0/(1.0+exp(-sum/13.0));
    }    

    //! Inactivation parameter for I_K
    double h_K1_infinity;
//    double t_h_K1;

    sum = E_M+110.0;
    if (sum < 0.0)
    {
      h_K1_infinity = 1.0/(1.0+exp( sum/18.0));
    }
    else
    {
      h_K1_infinity = exp( -sum/18.0)/(exp(-sum/18.0)+1.0);
    }
 
    double I_K1 = g_K1_bar*(E_M-E_K)*m_K1_infinity*h_K1_infinity;
    sum = I_NA + I_K1; 
    //double g_leak_bar  = -sum/(E_M-E_L);

//    cout << "Given g_leak_bar = 2.0 mu S find E_L" << endl;
    double E_L = (2.0*E_M + sum)/2.0;
//    cout << "E_L = " << E_L << endl;

//    cout << "Let's check the equilibrium voltage:" << endl;
    double g_NA_inf = g_NA_bar*m_NA_infinity*m_NA_infinity*h_NA_infinity;
    double g_K1_inf = g_K1_bar*m_K1_infinity*h_K1_infinity;          
    double g_total = g_NA_inf+g_K1_inf+g_leak_bar;
    double ratio_NA = g_NA_inf/g_total;
    double ratio_K1 = g_K1_inf/g_total;
    double ratio_L = g_leak_bar/g_total;
    double E_Mcalc = ratio_NA*E_NA + ratio_K1*E_K + ratio_L*E_L;

//    cout << "Calculated E_M = " << E_Mcalc << endl;

    ///////////////////////////////////////////////////////////////////////////
    // Initial activations and inactivations:
    //   m_NA_infinity
    //   h_NA_infinity
    //   m_K1_infinity
    //   h_K1_infinity
    ///////////////////////////////////////////////////////////////////////////

//    cout << "Initial activation and inactivations are" << endl;
//    cout << "m_NA(0) = " << m_NA_infinity << endl;
//    cout << "h_NA(0) = " << h_NA_infinity << endl;
//    cout << "m_K1(0) = " << m_K1_infinity << endl;
//    cout << "h_K1(0) = " << h_K1_infinity << endl;

    q[0] = m_NA_infinity;
    q[1] = h_NA_infinity;
    q[2] = m_K1_infinity;
    q[3] = h_K1_infinity;

    E_L = -10.0;        
    return E_L;
  }


/*
  void run_plot (Widget w, XtPointer client_data, XtPointer call_data)
  {
    XmPushButtonCallbackStruct *P = (XmPushButtonCallbackStruct *)call_data;
    application_data *T = (application_data *)client_data;

    printf("Let's plot action potential:\n");
    int i;

    int size = 5;
    double tol,tinit,tend,hinit;

    //! Allocate memory for state variables
    DOUBLE_VECTOR yinit(size);
    DOUBLE_VECTOR y(size);
    double C = 14.28;

    ///////////////////////////////////////////////////////////////////////////
    // Allocate memory for dynamic parameters:
    //  p[0] g_NA_bar
    //  p[1] g_K1_bar
    //  p[2] g_leak_bar
    //  p[3] E_NA
    //  p[4] E_K
    //  p[5] E_L
    //
    //  q[0] = m_NA(0);
    //  q[1] = h_NA(0);
    //  q[2] = m_K1(0);
    //  q[3] = h_K1(0);
    ///////////////////////////////////////////////////////////////////////////

    DOUBLE_VECTOR p(6);
    DOUBLE_VECTOR q(4);

    //! Koch values
    double g_NA_bar = 2.0;
    double g_K1_bar = 0.120+1.17+0.084+1.2+0.054+0.02675+0.116; //2.7708
    double g_leak_bar   = .02;

    double NA_O = 491.0;
    double NA_I = 50.0;
    double K_O = 7.859;
    double K_I = 140.0;    

    double R = 8.31;           //! Rydberg's Constant
    double DegK = 276.0 + C;   //! Kelvin Temperature; use C degrees Celsius
    double F = 9.649e+4;       //! Faraday's constant
    double RTF = R*DegK/F*1e+3;

    double E_NA;
    E_NA = RTF*log(NA_O/NA_I);
    double E_K;
    E_K = RTF*log(K_O/K_I);    

    double E_M = -60.0;

    p[0] = g_NA_bar;
    p[1] = g_K1_bar;
    p[2] = g_leak_bar;
    p[3] = E_NA;
    p[4] = E_K;
    p[5] = 0.0;

    double E_L = rest(E_M,p,q);
    p[5] = E_L;

//    cout << "E_NA        =  " << E_NA << endl;
//    cout << "E_K         =  " << E_K << endl;     
//    cout << "E_M         =  " << E_M << endl;
//    cout << "E_L         =  " << E_L << endl;   
//    cout << "g_leak_bar  =  " << g_leak_bar << endl;        

    //! Set up initial conditions
    T->initial = new DOUBLE_VECTOR(size);     
    DOUBLE_VECTOR& INIT = *(T->initial);

    INIT[0] = E_M;
    INIT[1] = q[0];
    INIT[2] = q[1];
    INIT[3] = q[2];
    INIT[4] = q[3];

    yinit = INIT;
    y = INIT;
//    cout << "yinit = " << yinit << endl;

    ///////////////////////////////////////////////////////////////////////////
    // y vector assignments
    ///////////////////////////////////////////////////////////////////////////
    //  y[0] = V
    //  y[1] = m_NA
    //  y[2] = h_NA
    //  y[3] = m_K1
    //  y[4] = h_K1
    //
    //  plot_type = 0 ==> y[0]  range [-80,100]
    //  plot_type = 1 ==> y[1]  range [0,1] 
    //  plot_type = 2 ==> y[2]  range [0,1]
    //  plot_type = 3 ==> y[3]  range [0,1]
    //  plot_type = 4 ==> y[4]  range [0,6.0e-2] 
    //
    //  plot_type = 5  ==> alpha_m_NA     range [0,50]
    //  plot_type = 6  ==> beta_m_NA      range [0,50]
    //  plot_type = 7  ==> alpha_h_NA     range [0,1]
    //  plot_type = 8  ==> beta_h_NA      range [0,8]
    //  plot_type = 9  ==> m_NA_infinity  range [0,1]
    //  plot_type = 10 ==> h_NA_infinity  range [0,1]
    //  plot_type = 11 ==> m_K1_infinity  range [0,1]
    //  plot_type = 12 ==> h_K1_infinity  range [0,0.08]
    //  plot_type = 13  ==>t_m_NA         range [0,1]
    //  plot_type = 14 ==> t_h_NA         range [0,1]
    //  plot_type = 15 ==> t_m_K1         range [0,1]
    //  plot_type = 16 ==> t_h_K1         range [0,1]
    ///////////////////////////////////////////////////////////////////////////

    DOUBLE_VECTOR Low_Range(17);
    DOUBLE_VECTOR High_Range(17);
    Low_Range[0] = -80.0;
    High_Range[0] = 100.0;    
    for (i = 1; i < 5; ++i)
    {
      Low_Range[i] = 0.0;
      High_Range[i] = 1.0;
    }
    High_Range[4] = 6.0e-2; 
    for (i = 5; i < 7; ++i)
    {
     Low_Range[i] = 0.0;
     High_Range[i] = 50.0;
    }
    for (i = 7; i < 17; ++i)
    {
      Low_Range[i] = 0.0;
      High_Range[i] = 1.0;
    }

    High_Range[8] = 8.0;
    High_Range[12] = 0.08;  
    High_Range[14] = 1.0e+1;          
    int plot_type;

    //! Set up parameters for ode integration
    hinit          =   1.0e-6;
    tinit          =   0.00;
    tend           =  80.0;
    tol            =   1.0e-14;
    plot_type  = 0;
    double ymin;
    double ymax;
    for (i = 0; i < 17; ++i)
    {
      if (plot_type == i)
      {
        ymin = Low_Range[i];
        ymax = High_Range[i];
      }
    }
    T->PLOT->plot_type = plot_type;
    T->PLOT->x_minimum = tinit;
    T->PLOT->x_maximum = tend;
    T->PLOT->y_minimum = ymin;
    T->PLOT->y_maximum = ymax;    

//    cout << "Entering rkf5_plot code" << endl;
//    cout << "parameters = " << p << endl;
    oderkf5_plot(yinit,y,p,tol,tinit,tend,hinit,T,w);
//    cout << "Returned from oderkf5_plot call " << endl;
  }
*/

/*
  /*==========================================================
  /*                                                          
  /* Runga-Fehlberg order 5 method for plotting               
  /*                                                          
  /* yinit          = initial state                           
  /* y              = calculated state                        
  /* u              = control parameter                      
  /* tinit          = initial t                               
  /* tend           = final t                                 
  /* tol            = tolerance for step size changing        
  /* hinit          = initial step size                       
  /*==========================================================
    ///////////////////////////////////////////////////////////////////////////
  void oderkf5_plot (DOUBLE_VECTOR& yinit, DOUBLE_VECTOR& y, DOUBLE_VECTOR& p,
                     double tol, double tinit, double tend, double hinit,
                     application_data *T, Widget w)
  {
    //open a file for plot data
    ofstream fd2;
    fd2.open("mK1inf", ios::in);
    DOUBLE_VECTOR GATES(14);
    ///////////////////////////////////////////////////////////////////////////
    // Gates[0]  = alpha_m_NA;
    // Gates[1]  = beta_m_NA;
    // Gates[2]  = alpha_h_NA;
    // Gates[3]  = beta_h_NA;
    // Gates[4]  = m_NA_infinity;
    // Gates[5]  = h_NA_infinity;
    // Gates[6]  = m_K1_infinity;
    // Gates[7]  = h_K1_infinity;
    // Gates[8]  = t_m_NA;
    // Gates[9]  = t_h_NA; 
    // Gates[10] = t_m_K1;   
    // Gates[11] = t_h_K1; 
    // Gates[12] = I_NA;
    // Gates[13] = I_K1;
    ///////////////////////////////////////////////////////////////////////////
    int POINTS_PER_UNIT = 20;
    int NUMBER_PLOT_POINTS = int(tend)*POINTS_PER_UNIT;
    float HPLOT = (tend-tinit)/(float)NUMBER_PLOT_POINTS;
    double tfinal;
    int i,j,k,plot_diagnostic;
    plot_data *PLOT = (plot_data *)(T->POP->Outside_Data);
    int plot_type = PLOT->plot_type;
    cout << "parameters = " << p << endl;
    int FREQ = 200;

    //set diagnostic status level
    plot_diagnostic = 0;

    PLOT->draw_axis = 1;
    tfinal = tinit + HPLOT;
    DOUBLE_VECTOR y0 = yinit;
    gates(y[0],GATES,p);
    PLOT->first_time = tinit;
    if(0<=plot_type && plot_type<=4)
    {
      PLOT->first_ordinate = y[plot_type];
    }
    else if(5<=plot_type && plot_type<17)
    {
      PLOT->first_ordinate = GATES[plot_type-5];
    }
    fd2 << PLOT->first_time << " " << PLOT->first_ordinate << endl;  
    for (int count=0; count<NUMBER_PLOT_POINTS; ++ count)
    {
      int debug = 0;
      int display_counter = 1;
      int plot_counter = 1;
   
      //integrate from tinit to tfinal 
      oderkf5_parobjects(y0,y,p,tol,tinit,tfinal,hinit);
      gates(y[0],GATES,p);
      if( count%FREQ==0 || count < 5)
      {
        if(0<=plot_type && plot_type<=4)
        {
          cout << "(t,y[" << plot_type << "]) = " << tfinal  
               << "," << y[plot_type] << endl;
        }
        else if(5<=plot_type && plot_type<17)
        {
          cout << "(t,GATES[" << plot_type-5 << "]) = " << tfinal  
               << "," << GATES[plot_type-5] << endl;      
        }
      }

      PLOT->second_time = tfinal;
      if(0<=plot_type && plot_type<=4)
      {
        PLOT->second_ordinate = y[plot_type];
      }
      else if(5<=plot_type && plot_type<17)
      {
        PLOT->second_ordinate = GATES[plot_type-5];
      }        
      if(plot_diagnostic==1)
      {
        printf("first_time = %8.2f first_ordinate = %8.2f\n",
        PLOT->first_time,PLOT->first_ordinate);
        printf("second_time = %8.2f second_ordinate = %8.2f\n",
        PLOT->second_time,PLOT->second_ordinate);
      }
      fd2 << PLOT->second_time << " " << PLOT->second_ordinate << endl;
      T->POP->Image(w,T->POP);
      PLOT->draw_axis = 0;
      PLOT->first_time = PLOT->second_time;
      PLOT->first_ordinate = PLOT->second_ordinate;  
      //reset yinit
      y0 = y;
      tinit = tfinal;
      tfinal += HPLOT;
    }//plot points loop
    fd2.close();
    cout << "Finished with PLOT loop " << endl;
  } 
*/
} // namespace Neuron
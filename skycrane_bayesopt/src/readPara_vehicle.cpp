#include "readPara_vehicle.h"

readPara_vehicle::readPara_vehicle(ros::NodeHandle nh, bool showReadInfo){
    /*If you want to modify this code, please remember to modify the code when showReadInfo is false also!*/
    if(showReadInfo){
        ROS_INFO("reading the parameters of the vehicle");
        /*computer parameter*/
        if (nh.getParam("CPUcoreNumber", CPUcoreNumber))
            ROS_INFO("CPUcoreNumber: %d", CPUcoreNumber);
        else
            ROS_INFO("No parameters CPUcoreNumber");

        /*basic parameter*/
        if (nh.getParam("stateDOF", stateDOF))
            ROS_INFO("stateDOF: %d", stateDOF);
        else
            ROS_INFO("No parameters stateDOF");

        if (nh.getParam("observationDOF", observationDOF))
            ROS_INFO("observationDOF: %d", observationDOF);
        else
            ROS_INFO("No parameters observationDOF");

        if (nh.getParam("controlDOF", controlDOF))
            ROS_INFO("controlDOF: %d", controlDOF);
        else
            ROS_INFO("No parameters controlDOF");

        if (nh.getParam("bayesoptTotalSampleNumber", bayesoptTotalSampleNumber))
            ROS_INFO("bayesoptTotalSampleNumber: %d", bayesoptTotalSampleNumber);
        else
            ROS_INFO("No parameters bayesoptTotalSampleNumber");

        if (nh.getParam("beta", beta))
            ROS_INFO("beta: %f", beta);
        else
            ROS_INFO("No parameters beta");

        if (nh.getParam("g", g))
            ROS_INFO("g: %f", g);
        else
            ROS_INFO("No parameters g");

        if (nh.getParam("rho", rho))
            ROS_INFO("rho: %f", rho);
        else
            ROS_INFO("No parameters rho");

        if (nh.getParam("C_D", C_D))
            ROS_INFO("C_D: %f", C_D);
        else
            ROS_INFO("No parameters C_D");

        if (nh.getParam("m_b", m_b))
            ROS_INFO("m_b: %f", m_b);
        else
            ROS_INFO("No parameters m_b");

        if (nh.getParam("m_f", m_f))
            ROS_INFO("m_f: %f", m_f);
        else
            ROS_INFO("No parameters m_f");

        if (nh.getParam("w_b", w_b))
            ROS_INFO("w_b: %f", w_b);
        else
            ROS_INFO("No parameters w_b");

        if (nh.getParam("h_b", h_b))
            ROS_INFO("h_b: %f", h_b);
        else
            ROS_INFO("No parameters h_b");

        if (nh.getParam("d_b", d_b))
            ROS_INFO("d_b: %f", d_b);
        else
            ROS_INFO("No parameters d_b");

        if (nh.getParam("w_f", w_f))
            ROS_INFO("w_f: %f", w_f);
        else
            ROS_INFO("No parameters w_f");

        if (nh.getParam("h_f", h_f))
            ROS_INFO("h_f: %f", h_f);
        else
            ROS_INFO("No parameters h_f");

        if (nh.getParam("d_f", d_f))
            ROS_INFO("d_f: %f", d_f);
        else
            ROS_INFO("No parameters d_f");

        /*simulation time parameter*/
        if (nh.getParam("dt", dt))
            ROS_INFO("dt: %f", dt);
        else
            ROS_INFO("No parameters dt");

        if (nh.getParam("t_start", t_start))
            ROS_INFO("t_start: %f", t_start);
        else
            ROS_INFO("No parameters t_start");

        if (nh.getParam("t_end", t_end))
            ROS_INFO("t_end: %f", t_end);
        else
            ROS_INFO("No parameters t_end");

        /*initial state*/
        if (nh.getParam("z0", z0))
            ROS_INFO("z0: %f", z0);
        else
            ROS_INFO("No parameters z0");

        if (nh.getParam("zdot0", zdot0))
            ROS_INFO("zdot0: %f", zdot0);
        else
            ROS_INFO("No parameters zdot0");

        if (nh.getParam("theta0", theta0))
            ROS_INFO("theta0: %f", theta0);
        else
            ROS_INFO("No parameters theta0");

        if (nh.getParam("thetadot0", thetadot0))
            ROS_INFO("thetadot0: %f", thetadot0);
        else
            ROS_INFO("No parameters thetadot0");

        if (nh.getParam("xi0", xi0))
            ROS_INFO("xi0: %f", xi0);
        else
            ROS_INFO("No parameters xi0");

        if (nh.getParam("xidot0", xidot0))
            ROS_INFO("xidot0: %f", xidot0);
        else
            ROS_INFO("No parameters xidot0");

        if (nh.getParam("disturbance", disturbance)){
            if(disturbance.size() != stateDOF){
                ROS_INFO("disturbance size must equal to stateDOF");
            }
            else{
                ROS_INFO("got disturbance");
            }
        }

        if (nh.getParam("P0", P0))
            ROS_INFO("P0: %s", P0.c_str());
        else
            ROS_INFO("No parameters P0");

        if (nh.getParam("P0_BASE", P0_BASE))
            ROS_INFO("P0_BASE: %s", P0_BASE.c_str());
        else
            ROS_INFO("No parameters P0_BASE");

        /*Number of running simulators and EKF max step*/
        if (nh.getParam("Nsimruns", Nsimruns))
            ROS_INFO("Nsimruns: %d", Nsimruns);
        else
            ROS_INFO("No parameters Nsimruns");


        /*Parameter computed by the parameters above*/
        h_cm  = (m_b * h_b * 0.5 - m_f*h_f*0.5)/(m_b + m_f);
        ROS_INFO("h_cm: %f", h_cm);
        w_cm  = w_b/2.0;
        ROS_INFO("w_cm: %f", w_cm);
        Ieta  = (1.0/12.0)*(m_b*(w_b*w_b + h_b*h_b)+m_f*(w_f*w_f + h_f*h_f));
        ROS_INFO("Ieta: %f", Ieta);
        Aside = h_b*d_b + h_f*d_f;
        ROS_INFO("Aside: %f", Aside);
        Abot  = w_b*d_b + w_f*d_f;
        ROS_INFO("Abot: %f", Abot);
        length = (t_end - t_start)/dt;
        ROS_INFO("length: %d", length);
        maxIterations = length;
        ROS_INFO("maxIterations: %d", length);
        

        /*gorund truth vehicle noise parameter for simulator, optimizationChice and costChoice*/
        if (nh.getParam("dim_pn", dim_pn))
            ROS_INFO("dim_pn: %d", dim_pn);
        else
            ROS_INFO("No parameters dim_pn");
        
        if (nh.getParam("dim_on", dim_on))
            ROS_INFO("dim_on: %d", dim_on);
        else
            ROS_INFO("No parameters dim_on");

        if (nh.getParam("pnoise", pnoise)){
            if(pnoise.size() != dim_pn){
                ROS_INFO("process noise size must equal to dim_pn");
            }
            else{
                ROS_INFO("Got process noise");
            }
        }

        if (nh.getParam("onoise", onoise)){
            if(onoise.size() != dim_on){
                ROS_INFO("process noise size must equal to dim_on");
            }
            else{
                ROS_INFO("Got observation noise");
            }
        }

        if(nh.getParam("optimizationChoice",optimizationChoice))
            ROS_INFO("optimizationChoice is : %s", optimizationChoice.c_str());
        else
            ROS_INFO("no parameter optimizationChoice");

        if(nh.getParam("costChoice",costChoice))
            ROS_INFO("costChoice is : %s", costChoice.c_str());
        else
            ROS_INFO("no parameter costChoice");

        /*controller*/
        if (nh.getParam("Klin", Klin)){
            if(Klin.size() != controlDOF*stateDOF){
                ROS_WARN("Klin size must be equal to controlDOF*stateDOF");
            }
            else{
                ROS_INFO("Got Klin");
            }
        }
    }
    else{
        /*computer parameter*/
        nh.getParam("CPUcoreNumber", CPUcoreNumber);

        /*basic parameter*/
        nh.getParam("beta", beta);
        nh.getParam("g", g);
        nh.getParam("rho", rho);
        nh.getParam("C_D", C_D);
        nh.getParam("m_b", m_b);
        nh.getParam("m_f", m_f);
        nh.getParam("w_b", w_b);
        nh.getParam("h_b", h_b);
        nh.getParam("d_b", d_b);
        nh.getParam("w_f", w_f);
        nh.getParam("h_f", h_f);
        nh.getParam("d_f", d_f);
        nh.getParam("stateDOF", stateDOF);
        nh.getParam("observationDOF", observationDOF);
        nh.getParam("controlDOF", controlDOF);
        nh.getParam("bayesoptTotalSampleNumber", bayesoptTotalSampleNumber);

        /*simulation time parameter*/
        nh.getParam("dt", dt);
        nh.getParam("t_start", t_start);
        nh.getParam("t_end", t_end);

        /*initial state*/
        nh.getParam("z0", z0);
        nh.getParam("zdot0", zdot0);
        nh.getParam("theta0", theta0);
        nh.getParam("thetadot0", thetadot0);
        nh.getParam("xi0", xi0);
        nh.getParam("xidot0", xidot0);
        nh.getParam("P0", P0);
        nh.getParam("disturbance", disturbance);
        nh.getParam("P0_BASE", P0_BASE);

        /*Number of running simulators and EKF max step*/
        nh.getParam("Nsimruns", Nsimruns);

        /*Parameter computed by the parameters above*/
        h_cm  = (m_b * h_b * 0.5 - m_f*h_f*0.5)/(m_b + m_f);
        w_cm  = w_b/2.0;
        Ieta  = (1.0/12.0)*(m_b*(w_b*w_b + h_b*h_b)+m_f*(w_f*w_f + h_f*h_f));
        Aside = h_b*d_b + h_f*d_f;
        Abot  = w_b*d_b + w_f*d_f;
        length = (t_end - t_start)/dt;
        maxIterations = length;

        /*gorund truth vehicle noise parameter for simulator*/
        nh.getParam("dim_pn", dim_pn);
        nh.getParam("dim_on", dim_on);
        nh.getParam("pnoise", pnoise);
        nh.getParam("onoise", onoise);

        /*optimization choice*/
        nh.getParam("optimizationChoice",optimizationChoice); //optimize "processNoise" or "observationNoise" or "both"

        /*cost choice, JNEES or JNIS*/
        nh.getParam("costChoice",costChoice);

        /*controller*/
        nh.getParam("Klin", Klin);
    }
}

readPara_vehicle::readPara_vehicle(){};
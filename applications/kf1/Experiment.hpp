#ifndef __EXPERIMENT_HPP__
#define __EXPERIMENT_HPP__

// Each experiment consists of a set of trials.

class Experiment
{
public:
  Experiment(int experimentNumber);

  void run();
  
private:

  // The experiment number
  int _experimentNumber;
};



#endif //  __EXPERIMENT_HPP__

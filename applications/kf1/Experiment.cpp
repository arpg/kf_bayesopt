#include "Experiment.hpp"
#include "Trial.hpp"
#include "ThreadPool.h"

// I hate lambdas

static double runExperiment(int i)
{
  Parameters correctParameters(1, 1, 1);
  Parameters wrongParameters(i, 1 + i, 1);
  Trial t(i, correctParameters, wrongParameters);
  t.run();
  return t.getAverageNEES();
}

Experiment::Experiment(int experimentNumber) : _experimentNumber(experimentNumber)
{
}

void Experiment::run()
{
  // Create a thread pool
  ThreadPool pool(4);

  // Now launch a bunch of instances; we could change these to use different parameters
  std::vector< std::future<double> > results;

  for(int i = 0; i < 100; ++i)
    {
      results.emplace_back(
			   pool.enqueue([i]
					{
					  Parameters correctParameters(1, 1, 1);
					  Parameters wrongParameters(1, 1 + i, 1);
					  Trial t(i, correctParameters, wrongParameters);
					  t.run();
					  return t.getAverageNEES();
					}
					)
			   );
	}

  for(auto && result: results)
    std::cout << result.get() << std::endl;
}

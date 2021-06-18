#include <iostream>
#include <list>
#include <vector>

#include "Module.h"
int main()
{
  AS::Module module;
  module.initNodeMat(15, 29);
  module.initStartPoint(1, 1);
  module.initEndPoint(14, 17);
  module.appendObsPoint(2, 7);
  module.appendObsPoint(2, 6);
  module.appendObsPoint(2, 4);
  module.appendObsPoint(2, 5);
  module.appendObsPoint(3, 4);
  module.appendObsPoint(3, 5);
  module.appendObsPoint(0, 17);
  module.appendObsPoint(1, 17);
  module.appendObsPoint(3, 15);

  module.appendObsPoint(3, 16);
  module.appendObsPoint(3, 17);

  module.appendObsPoint(3, 18);

  module.runAStar();
  return 0;
}

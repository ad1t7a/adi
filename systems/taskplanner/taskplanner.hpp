#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"

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


namespace adi {
    namespace systems {
        class TaskPlanner {
            public:
                ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TaskPlanner);
                
                //! constructor
                TaskPlanner();

                //! destructor
                ~TaskPlanner();
        };
    }
}
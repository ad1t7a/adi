
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
#ifndef ACTION_ROT
#define ACTION_ROT
#include "STL_interface.hh"
#include "init/init_function.hh"
#include "init/init_matrix.hh"
#include "init/init_vector.hh"
#include "utilities.h"
#include <string>

using namespace std;

template <class Interface> class Action_rot {

public:
  // Ctor
  BTL_DONT_INLINE Action_rot(int size) : _size(size) {
    MESSAGE("Action_rot Ctor");

    // STL matrix and vector initialization
    typename Interface::stl_matrix tmp;
    init_vector<pseudo_random>(A_stl, _size);
    init_vector<pseudo_random>(B_stl, _size);

    // generic matrix and vector initialization
    Interface::vector_from_stl(A_ref, A_stl);
    Interface::vector_from_stl(A, A_stl);
    Interface::vector_from_stl(B_ref, B_stl);
    Interface::vector_from_stl(B, B_stl);
  }

  // invalidate copy ctor
  Action_rot(const Action_rot &) {
    INFOS("illegal call to Action_rot Copy Ctor");
    exit(1);
  }

  // Dtor
  BTL_DONT_INLINE ~Action_rot(void) {
    MESSAGE("Action_rot Dtor");
    Interface::free_vector(A);
    Interface::free_vector(B);
    Interface::free_vector(A_ref);
    Interface::free_vector(B_ref);
  }

  // action name
  static inline std::string name(void) { return "rot_" + Interface::name(); }

  double nb_op_base(void) { return 6.0 * _size; }

  BTL_DONT_INLINE void initialize(void) {
    Interface::copy_vector(A_ref, A, _size);
    Interface::copy_vector(B_ref, B, _size);
  }

  BTL_DONT_INLINE void calculate(void) {
    BTL_ASM_COMMENT("#begin rot");
    Interface::rot(A, B, 0.5, 0.6, _size);
    BTL_ASM_COMMENT("end rot");
  }

  BTL_DONT_INLINE void check_result(void) {
    // calculation check
    //     Interface::vector_to_stl(X,resu_stl);

    //     STL_interface<typename
    //     Interface::real_type>::rot(A_stl,B_stl,X_stl,_size);

    //     typename Interface::real_type error=
    //       STL_interface<typename
    //       Interface::real_type>::norm_diff(X_stl,resu_stl);

    //     if (error>1.e-3){
    //       INFOS("WRONG CALCULATION...residual=" << error);
    //       exit(0);
    //     }
  }

private:
  typename Interface::stl_vector A_stl;
  typename Interface::stl_vector B_stl;

  typename Interface::gene_vector A_ref;
  typename Interface::gene_vector B_ref;

  typename Interface::gene_vector A;
  typename Interface::gene_vector B;

  int _size;
};

#endif
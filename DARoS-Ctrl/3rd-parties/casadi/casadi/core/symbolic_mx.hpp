/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2023 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            KU Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


#ifndef CASADI_SYMBOLIC_MX_HPP
#define CASADI_SYMBOLIC_MX_HPP

#include "mx_node.hpp"

/// \cond INTERNAL

namespace casadi {
  /** \brief Represents a symbolic MX

      \author Joel Andersson
      \date 2010
      A regular user is not supposed to work with this Node class.
      This user can call MX(name, n, m) directly.

      \identifier{1i0} */
  class CASADI_EXPORT SymbolicMX : public MXNode {
  public:

    /** \brief  Constructors

        \identifier{1i1} */
    explicit SymbolicMX(const std::string& name, casadi_int nrow=1, casadi_int ncol=1);

    /** \brief  Constructors

        \identifier{1i2} */
    explicit SymbolicMX(const std::string& name, const Sparsity & sp);

    /// Destructor
    ~SymbolicMX() override {}

    /** \brief  Print expression

        \identifier{1i3} */
    std::string disp(const std::vector<std::string>& arg) const override;

    /// Evaluate the function numerically
    int eval(const double** arg, double** res, casadi_int* iw, double* w) const override;

    /// Evaluate the function symbolically (SX)
    int eval_sx(const SXElem** arg, SXElem** res, casadi_int* iw, SXElem* w) const override;

    /** \brief  Evaluate symbolically (MX)

        \identifier{1i4} */
    void eval_mx(const std::vector<MX>& arg, std::vector<MX>& res) const override;

    /** \brief Calculate forward mode directional derivatives

        \identifier{1i5} */
    void ad_forward(const std::vector<std::vector<MX> >& fseed,
                         std::vector<std::vector<MX> >& fsens) const override;

    /** \brief Calculate reverse mode directional derivatives

        \identifier{1i6} */
    void ad_reverse(const std::vector<std::vector<MX> >& aseed,
                         std::vector<std::vector<MX> >& asens) const override;

    /** \brief  Propagate sparsity forward

        \identifier{1i7} */
    int sp_forward(const bvec_t** arg, bvec_t** res, casadi_int* iw, bvec_t* w) const override;

    /** \brief  Propagate sparsity backwards

        \identifier{1i8} */
    int sp_reverse(bvec_t** arg, bvec_t** res, casadi_int* iw, bvec_t* w) const override;

    /** \brief  Get the name

        \identifier{1i9} */
    const std::string& name() const override;

    /** \brief Get the operation

        \identifier{1ia} */
    casadi_int op() const override { return OP_PARAMETER;}

    /** \brief  Check if valid function input

        \identifier{1ib} */
    bool is_valid_input() const override { return true;}

    /** \brief Detect duplicate symbolic expressions

        \identifier{1ic} */
    bool has_duplicates() const override;

    /** \brief Reset the marker for an input expression

        \identifier{1id} */
    void reset_input() const override;

    /** \brief Serialize an object without type information

        \identifier{1ie} */
    void serialize_body(SerializingStream& s) const override;

    /** \brief Deserialize without type information

        \identifier{1if} */
    static MXNode* deserialize(DeserializingStream& s) { return new SymbolicMX(s); }

  protected:
    /** \brief Deserializing constructor

        \identifier{1ig} */
    explicit SymbolicMX(DeserializingStream& s);

    // Name of the variable
    std::string name_;
  };

} // namespace casadi

/// \endcond

#endif // CASADI_SYMBOLIC_MX_HPP

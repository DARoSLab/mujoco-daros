/*!
 * @file ContactSpringDamper.cpp
 * @brief Spring Damper based Contact dynamics
 */

#include "ContactSpringDamper.h"

/*!
 * compute contact forces coinciding to the contact and update external force
 * in a dynamics model.
 */
template <typename T>
void ContactSpringDamper<T>::UpdateExternalForces(T K, T D, T dt) {
  this->_nContact = this->_CheckContact();
  for (size_t i(0); i < _nGC; ++i) {
    // first assume there's no contact, so the ground "springs back"
    deflectionRate[i] = (-K / D) * _tangentialDeflections[i];
  }
  if (this->_nContact > 0) {
    _groundContactWithOffset(K, D);
    for (size_t i(0); i < this->_nContact; ++i) {
      this->_model->_externalForces.at(this->_model->_gcParent[this->_idx_list[i]]) +=
          forceToSpatialForce(this->_cp_force_list[this->_idx_list[i]],
                              this->_cp_pos_list[i]);
    }
    static int count(0);
    ++count;
    if (count > 5) {
      // exit(0);
    }
  }

  for (size_t i(0); i < _nGC; ++i) {
    _tangentialDeflections[i] += dt * deflectionRate[i];
  }
}

/*!
 * subfunction computing contact forces
 */
template <typename T>
void ContactSpringDamper<T>::_groundContactWithOffset(T K, T D) {
  for (size_t i = 0; i < this->_nContact; i++) {
    Vec3<T> v =
        this->_cp_frame_list[i].transpose() *
        this->_model->_vGC[this->_idx_list[i]];  // velocity in plane coordinates
    T z = this->_cp_penetration_list[i];       // the penetration into the ground
    T zd = v[2];                             // the penetration velocity
    T zr = std::sqrt(std::max(T(0), -z));  // sqrt penetration into the ground,
                                           // or zero if we aren't penetrating
    T normalForce =
        zr *
        (-K * z - D * zd);  // normal force is spring-damper * sqrt(penetration)

    // set output force to zero for now.
    this->_cp_local_force_list[i][0] = 0;
    this->_cp_local_force_list[i][1] = 0;
    this->_cp_local_force_list[i][2] = 0;

    if (normalForce > 0) {
      this->_cp_local_force_list[i][2] =
          normalForce;  // set the normal force. This is in the plane's
                        // coordinates for now

      // first, assume sticking
      // this means the tangential deformation happens at the speed of the foot.
      deflectionRate[this->_idx_list[i]] = v.template topLeftCorner<2, 1>();
      Vec2<T> tangentialSpringForce =
          K * zr *
          _tangentialDeflections[this->_idx_list[i]];  // tangential force due to
                                                     // "spring"
      Vec2<T> tangentialForce =
          -tangentialSpringForce -
          D * zr * deflectionRate[this->_idx_list[i]];  // add damping to get
                                                      // total tangential

      // check for slipping:
      T slipForce = this->_cp_mu_list[i] *
                    normalForce;  // maximum force magnitude without slipping
      T tangentialForceMagnitude =
          tangentialForce
              .norm();  // actual force magnitude if we assume sticking
      T r = tangentialForceMagnitude / slipForce;  // ratio of force/max_force

      if (r > 1) {
        // we are slipping.
        tangentialForce =
            tangentialForce / r;  // adjust tangential force to avoid slipping
        deflectionRate[this->_idx_list[i]] =
            -(tangentialForce + tangentialSpringForce) / (D * zr);
      }
      // set forces
      this->_cp_local_force_list[i][0] = tangentialForce[0];
      this->_cp_local_force_list[i][1] = tangentialForce[1];
    }
    // move back into robot frame
    this->_cp_force_list[this->_idx_list[i]] =
        this->_cp_frame_list[i] * this->_cp_local_force_list[i];
  }
}

template class ContactSpringDamper<double>;
template class ContactSpringDamper<float>;

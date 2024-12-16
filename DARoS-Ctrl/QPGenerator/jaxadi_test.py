from jaxadi import convert, translate
import jax
import casadi as ca
import jax.numpy as jnp


qp_func = ca.Function.load("staccatoe_horizon_1_real_double_qp_funcs_jac_G_z.casadi")
# translate the casadi function to jax
print("Translating the casadi function to jax")
# print(translate(qp_func, add_import=True, add_jit=True))

print(f"num of instructions: {qp_func.n_instructions()}")

# convert the casadi function to jax
jax_fn = convert(qp_func, compile=False, num_threads=20)

# print the jax function
print("Printing the jax function")
print(jax_fn)

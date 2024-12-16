import textwrap
from casadi import *
import casadi as ca

OP_JAX_DICT = {
    OP_ASSIGN:              "\n        work = work.at[%d].set(work[%d])",
    OP_ADD:                 "\n        work = work.at[%d].set(work[%d] + work[%d])",
    OP_SUB:                 "\n        work = work.at[%d].set(work[%d] - work[%d])",
    OP_MUL:                 "\n        work = work.at[%d].set(work[%d] * work[%d])",
    OP_DIV:                 "\n        work = work.at[%d].set(work[%d] / work[%d])",
    OP_NEG:                 "\n        work = work.at[%d].set(-work[%d])",
    OP_EXP:                 "\n        work = work.at[%d].set(jnp.exp(work[%d]))",
    OP_LOG:                 "\n        work = work.at[%d].set(jnp.log(work[%d]))",
    OP_POW:                 "\n        work = work.at[%d].set(jnp.power(work[%d], work[%d]))",
    OP_CONSTPOW:            "\n        work = work.at[%d].set(jnp.power(work[%d], work[%d]))",
    OP_SQRT:                "\n        work = work.at[%d].set(jnp.sqrt(work[%d]))",
    OP_SQ:                  "\n        work = work.at[%d].set(work[%d] * work[%d])",
    OP_TWICE:               "\n        work = work.at[%d].set(2. * work[%d])",
    OP_SIN:                 "\n        work = work.at[%d].set(jnp.sin(work[%d]))",
    OP_COS:                 "\n        work = work.at[%d].set(jnp.cos(work[%d]))",
    OP_TAN:                 "\n        work = work.at[%d].set(jnp.tan(work[%d]))",
    OP_ASIN:                "\n        work = work.at[%d].set(jnp.arcsin(work[%d]))",
    OP_ACOS:                "\n        work = work.at[%d].set(jnp.arccos(work[%d]))",
    OP_ATAN:                "\n        work = work.at[%d].set(jnp.arctan(work[%d]))",
    OP_LT:                  "\n        work = work.at[%d].set(work[%d] < work[%d])",
    OP_LE:                  "\n        work = work.at[%d].set(work[%d] <= work[%d])",
    OP_EQ:                  "\n        work = work.at[%d].set(work[%d] == work[%d])",
    OP_NE:                  "\n        work = work.at[%d].set(work[%d] != work[%d])",
    OP_NOT:                 "\n        work = work.at[%d].set(jnp.logical_not(work[%d]))",
    OP_AND:                 "\n        work = work.at[%d].set(jnp.logical_and(work[%d], work[%d]))",
    OP_OR:                  "\n        work = work.at[%d].set(jnp.logical_or(work[%d], work[%d]))",
    OP_FLOOR:               "\n        work = work.at[%d].set(jnp.floor(work[%d]))",
    OP_CEIL:                "\n        work = work.at[%d].set(jnp.ceil(work[%d]))",
    OP_FMOD:                "\n        work = work.at[%d].set(jnp.mod(work[%d], work[%d]))",
    OP_FABS:                "\n        work = work.at[%d].set(jnp.abs(work[%d]))",
    OP_SIGN:                "\n        work = work.at[%d].set(jnp.sign(work[%d]))",
    OP_COPYSIGN:            "\n        work = work.at[%d].set(jnp.copysign(work[%d], work[%d]))",
    OP_IF_ELSE_ZERO:        "\n        work = work.at[%d].set(jnp.where(work[%d] == 0, 0, work[%d]))",
    OP_ERF:                 "\n        work = work.at[%d].set(jax.scipy.special.erf(work[%d]))",
    OP_FMIN:                "\n        work = work.at[%d].set(jnp.minimum(work[%d], work[%d]))",
    OP_FMAX:                "\n        work = work.at[%d].set(jnp.maximum(work[%d], work[%d]))",
    OP_INV:                 "\n        work = work.at[%d].set(1. / work[%d])",
    OP_SINH:                "\n        work = work.at[%d].set(jnp.sinh(work[%d]))",
    OP_COSH:                "\n        work = work.at[%d].set(jnp.cosh(work[%d]))",
    OP_TANH:                "\n        work = work.at[%d].set(jnp.tanh(work[%d]))",
    OP_ASINH:               "\n        work = work.at[%d].set(jnp.arcsinh(work[%d]))",
    OP_ACOSH:               "\n        work = work.at[%d].set(jnp.arccosh(work[%d]))",
    OP_ATANH:               "\n        work = work.at[%d].set(jnp.arctanh(work[%d]))",
    OP_ATAN2:               "\n        work = work.at[%d].set(jnp.arctan2(work[%d], work[%d]))",
    OP_CONST:               "\n        work = work.at[%d].set(%f)",
    OP_INPUT:               "\n        work = work.at[%d].set(inputs[%d][%d])",
    OP_OUTPUT:              "\n        outputs = outputs.at[%d, %d].set(work[%d])",
}

def generateJaxCode(f, filepath=None):
    codegen_filepath = filepath + "_jax.py"
    codegen_file = open(codegen_filepath, "w+")
    codegen_strings = {}

    # * Parse CasADi function
    n_instr = f.n_instructions()
    n_in = f.n_in()
    n_out = f.n_out()
    nnz_in = [f.nnz_in(i) for i in range(n_in)]
    nnz_out = [f.nnz_out(i) for i in range(n_out)]
    n_w = f.sz_w()

    INSTR_LIMIT = n_instr  
    input_idx = [f.instruction_input(i) for i in range(INSTR_LIMIT)]
    output_idx = [f.instruction_output(i) for i in range(INSTR_LIMIT)]
    operations = [f.instruction_id(i) for i in range(INSTR_LIMIT)]
    const_instr = [f.instruction_constant(i) for i in range(INSTR_LIMIT)]

    # * Codegen for const declarations and indices
    codegen_strings['header'] = "# ! AUTOMATICALLY GENERATED CODE FOR CUSADI\n"
    codegen_strings['includes'] = textwrap.dedent(
    '''
    import jax
    import jax.numpy as jnp

    ''')
    codegen_strings["nnz_in"] = f"nnz_in = jnp.array([{','.join(map(str, nnz_in))}])\n"
    codegen_strings["nnz_out"] = f"nnz_out = jnp.array([{','.join(map(str, nnz_out))}])\n"
    codegen_strings["n_w"] = f"n_w = {n_w}\n\n"

    # * Codegen for JAX
    str_operations = "@jax.jit\n"
    str_operations += f"def evaluate_{f.name()}(outputs, inputs, work):\n"

    for k in range(INSTR_LIMIT):
        op = operations[k]
        o_idx = output_idx[k]
        i_idx = input_idx[k]
        if op == OP_CONST:
            str_operations += OP_JAX_DICT[op] % (o_idx[0], const_instr[k])
        elif op == OP_INPUT:
            str_operations += OP_JAX_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])
        elif op == OP_OUTPUT:
            str_operations += OP_JAX_DICT[op] % (o_idx[0], o_idx[1], i_idx[0])
        elif op == OP_SQ:
            str_operations += OP_JAX_DICT[op] % (o_idx[0], i_idx[0], i_idx[0])
        elif OP_JAX_DICT[op].count("%d") == 3:
            str_operations += OP_JAX_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])
        elif OP_JAX_DICT[op].count("%d") == 2:
            str_operations += OP_JAX_DICT[op] % (o_idx[0], i_idx[0])
        else:
            raise Exception('Unknown CasADi operation: ' + str(op))

    str_operations += "\n    return outputs\n"  # Add newline and proper indentation
    codegen_strings['jax_operations'] = str_operations

    # * Write codegen to file
    for cg_str in codegen_strings.values():
        codegen_file.write(cg_str)
    codegen_file.close()
    print("JAX codegen complete for CasADi function: ", f.name())


if __name__ == "__main__":
    # * Load CasADi function
    filepath = "staccatoe_horizon_5_real_double_qp_funcs.casadi"
    f = ca.Function.load(filepath)
    generateJaxCode(f, filepath.split(".")[0])
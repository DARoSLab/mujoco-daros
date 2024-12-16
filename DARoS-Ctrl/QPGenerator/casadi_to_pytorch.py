import textwrap
from casadi import *
import casadi as ca
OP_PYTORCH_DICT = {
    OP_ASSIGN:              "\n        work[%d] = work[%d]",
    OP_ADD:                 "\n        work[%d] = work[%d] + work[%d]",
    OP_SUB:                 "\n        work[%d] = work[%d] - work[%d]",
    OP_MUL:                 "\n        work[%d] = work[%d] * work[%d]",
    OP_DIV:                 "\n        work[%d] = work[%d] / work[%d]",
    OP_NEG:                 "\n        work[%d] = -work[%d]",
    OP_EXP:                 "\n        work[%d] = torch.exp(work[%d])",
    OP_LOG:                 "\n        work[%d] = torch.log(work[%d])",
    OP_POW:                 "\n        work[%d] = torch.pow(work[%d], work[%d])",
    OP_CONSTPOW:            "\n        work[%d] = torch.pow(work[%d], work[%d])",
    OP_SQRT:                "\n        work[%d] = torch.sqrt(work[%d])",
    OP_SQ:                  "\n        work[%d] = work[%d] * work[%d]",
    OP_TWICE:               "\n        work[%d] = 2.*(work[%d])",
    OP_SIN:                 "\n        work[%d] = torch.sin(work[%d])",
    OP_COS:                 "\n        work[%d] = torch.cos(work[%d])",
    OP_TAN:                 "\n        work[%d] = torch.tan(work[%d])",
    OP_ASIN:                "\n        work[%d] = torch.asin(work[%d])",
    OP_ACOS:                "\n        work[%d] = torch.acos(work[%d])",
    OP_ATAN:                "\n        work[%d] = torch.atan(work[%d])",
    OP_LT:                  "\n        work[%d] = work[%d] < work[%d]",
    OP_LE:                  "\n        work[%d] = work[%d] <= work[%d]",
    OP_EQ:                  "\n        work[%d] = work[%d] == work[%d]",
    OP_NE:                  "\n        work[%d] = work[%d] != work[%d]",
    OP_NOT:                 "\n        work[%d] = torch.logical_not(work[%d])",
    OP_AND:                 "\n        work[%d] = torch.logical_and(work[%d], work[%d])",
    OP_OR:                  "\n        work[%d] = torch.logical_or(work[%d], work[%d])",
    OP_FLOOR:               "\n        work[%d] = torch.floor(work[%d])",
    OP_CEIL:                "\n        work[%d] = torch.ceil(work[%d])",
    OP_FMOD:                "\n        work[%d] = torch.fmod(work[%d], work[%d])",
    OP_FABS:                "\n        work[%d] = torch.abs(work[%d])",
    OP_SIGN:                "\n        work[%d] = torch.sign(work[%d])",
    OP_COPYSIGN:            "\n        work[%d] = torch.copysign(work[%d], work[%d])",
    OP_IF_ELSE_ZERO:        "\n        work[%d] = (work[%d] == 0) ? 0 : work[%d]",
    OP_ERF:                 "\n        work[%d] = torch.erf(work[%d])",
    OP_FMIN:                "\n        work[%d] = torch.fmin(work[%d], work[%d])",
    OP_FMAX:                "\n        work[%d] = torch.fmax(work[%d], work[%d])",
    OP_INV:                 "\n        work[%d] = 1./(work[%d])",
    OP_SINH:                "\n        work[%d] = torch.sinh(work[%d])",
    OP_COSH:                "\n        work[%d] = torch.cosh(work[%d])",
    OP_TANH:                "\n        work[%d] = torch.tanh(work[%d])",
    OP_ASINH:               "\n        work[%d] = torch.asinh(work[%d])",
    OP_ACOSH:               "\n        work[%d] = torch.acosh(work[%d])",
    OP_ATANH:               "\n        work[%d] = torch.atanh(work[%d])",
    OP_ATAN2:               "\n        work[%d] = torch.atan2(work[%d], work[%d])",
    OP_CONST:               "\n        work[%d] = %f",
    OP_INPUT:               "\n        work[%d] = inputs[%d][%d]",
    OP_OUTPUT:              "\n        outputs[%d][%d] = work[%d]",
}

def generatePytorchCode(f, filepath=None):

    codegen_filepath = filepath + "_pytorch.py"
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
    import torch

    ''')
    codegen_strings["nnz_in"] = f"nnz_in = [{','.join(map(str, nnz_in))}]\n"
    codegen_strings["nnz_out"] = f"nnz_out = [{','.join(map(str, nnz_out))}]\n"
    codegen_strings["n_w"] = f"n_w = {n_w}\n\n"

    # * Codegen for Pytorch
    str_operations = "@torch.compile\n"
    str_operations += f"def evaluate_{f.name()}(outputs, inputs, work):"

    for k in range(INSTR_LIMIT):
        op = operations[k]
        o_idx = output_idx[k]
        i_idx = input_idx[k]
        if op == OP_CONST:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], const_instr[k])
        elif op == OP_INPUT:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])
        elif op == OP_OUTPUT:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], o_idx[1], i_idx[0])
        elif op == OP_SQ:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[0])
        elif OP_PYTORCH_DICT[op].count("%d") == 3:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0], i_idx[1])
        elif OP_PYTORCH_DICT[op].count("%d") == 2:
            str_operations += OP_PYTORCH_DICT[op] % (o_idx[0], i_idx[0])
        else:
            raise Exception('Unknown CasADi operation: ' + str(op))

    codegen_strings['pytorch_operations'] = str_operations

    # * Write codegen to file
    for cg_str in codegen_strings.values():
        codegen_file.write(cg_str)
    codegen_file.close()
    print("Pytorch codegen complete for CasADi function: ", f.name())


if __name__ == "__main__":
    # * Load CasADi function
    filepath = "staccatoe_horizon_5_real_double_qp_funcs.casadi"
    f = ca.Function.load(filepath)
    generatePytorchCode(f, filepath.split(".")[0])
import sys
import traceback

print("Starting script...")

try:
    import jax
    import jax.numpy as jnp
    print("JAX imported successfully.")
    print(f"JAX version: {jax.__version__}")
    print(f"JAX devices: {jax.devices()}")
    
    # Check if GPU is available
    if len(jax.devices('gpu')) == 0:
        print("No GPU found. Please ensure CUDA is properly installed and configured.")
        sys.exit(1)
    else:
        print(f"GPU found: {jax.devices('gpu')[0]}")
        
except ImportError as e:
    print(f"Error importing JAX: {e}")
    sys.exit(1)

try:
    from staccatoe_horizon_5_real_double_qp_funcs_jax import evaluate_staccatoe_horizon_5_real_double_qp_funcs, nnz_in, nnz_out, n_w
    print("Successfully imported the generated function and constants.")
except ImportError as e:
    print(f"Error importing generated function: {e}")
    sys.exit(1)

def test_jax_function():
    print("Starting test_jax_function...")
    
    # Set random seed for reproducibility
    key = jax.random.PRNGKey(0)

    print(f"nnz_in: {nnz_in}")
    print(f"nnz_out: {nnz_out}")
    print(f"n_w: {n_w}")

    # Create sample inputs
    inputs = [jax.random.normal(key, (size,)) for size in nnz_in]
    print(f"Created {len(inputs)} input arrays.")

    # Initialize outputs and work arrays
    outputs = jnp.zeros((len(nnz_out), jnp.max(nnz_out)))
    work = jnp.zeros(n_w)
    print(f"Initialized outputs array with shape {outputs.shape}")
    print(f"Initialized work array with shape {work.shape}")

    # Move data to GPU
    inputs = [jax.device_put(inp, jax.devices('gpu')[0]) for inp in inputs]
    outputs = jax.device_put(outputs, jax.devices('gpu')[0])
    work = jax.device_put(work, jax.devices('gpu')[0])
    print("Moved data to GPU.")

    # Compile the function for GPU
    jit_evaluate = jax.jit(evaluate_staccatoe_horizon_5_real_double_qp_funcs, backend='gpu')
    print("Compiled function for GPU.")

    # Run the function
    print("Calling evaluate_staccatoe_horizon_5_real_double_qp_funcs...")
    result = jit_evaluate(outputs, inputs, work)
    print("Function call completed.")

    # Print results
    print("\nInputs:")
    for i, inp in enumerate(inputs):
        print(f"Input {i}: shape {inp.shape}, first few values: {inp[:5]}")

    print("\nOutputs:")
    for i, out in enumerate(result):
        print(f"Output {i}: shape {out.shape}, first few non-zero values: {out[:nnz_out[i]][:5]}")

    print("\nTest completed successfully.")

if __name__ == "__main__":
    try:
        test_jax_function()
    except Exception as e:
        print(f"An error occurred during testing: {e}")
        print("Traceback:")
        traceback.print_exc()
        sys.exit(1)
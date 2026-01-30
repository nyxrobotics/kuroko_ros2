import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    raise RuntimeError(
        "Template script: implement SDF->USD export for your Isaac Sim installation. "
        f"Input SDF: {args.sdf} Output: {args.out}"
    )

if __name__ == "__main__":
    main()

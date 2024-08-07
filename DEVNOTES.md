# Notes

## Instruction for Updating the IDF Development Environment

```sh
# In project directory
idf.py fullclean

# In IDF directory:
# 1. Checkout to the branch with the newer version.
# 2.
git submodule update --init --recursive
# 3.
python3 tools/idf_tools.py install
# 4.
python3 tools/idf_tools.py uninstall --remove-archives
```

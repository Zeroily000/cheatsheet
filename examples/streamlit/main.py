import streamlit as st
from streamlit.web import cli as stcli
import sys

def main():
  st.write("""
    # My first app
    Hello *world!*
    """)

if __name__ == '__main__':
  print(st.runtime.exists())
  if st.runtime.exists():
    main()
  else:
    sys.argv = ["streamlit", "run", sys.argv[0]]
    sys.exit(stcli.main())

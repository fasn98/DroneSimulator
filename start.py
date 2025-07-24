#!/usr/bin/env python3
"""
Simple deployment entry point for Replit.
"""

if __name__ == '__main__':
    import os
    import sys
    
    # Add current directory to path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    
    # Import and run the main application
    from run import main
    main()
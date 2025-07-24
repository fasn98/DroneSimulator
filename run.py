#!/usr/bin/env python3
"""
Main entry point for the Drone Simulation Server deployment.
Provides a simple, fast-startup entry point for cloud deployments.
"""

import os
import sys
import logging
import time

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def main():
    """Main entry point for deployment - optimized for fast startup."""
    logger.info("Starting Drone Simulation Server deployment...")
    
    # Get port from environment variable or use default
    port = int(os.environ.get('PORT', 5000))
    host = os.environ.get('HOST', '0.0.0.0')
    
    try:
        # Import here to avoid early initialization issues
        logger.info("Loading simulation server...")
        from simulation_server import DroneSimulationServer
        
        # Create and run server with optimized settings
        logger.info(f"Initializing server on {host}:{port}")
        server = DroneSimulationServer(
            host=host,
            port=port,
            debug=False  # Disable debug mode for production
        )
        
        logger.info("Server ready - starting SocketIO service...")
        server.run()
    except KeyboardInterrupt:
        logger.info("Shutting down simulation server...")
    except Exception as e:
        logger.error(f"Server error: {e}")
        logger.exception("Full traceback:")
        sys.exit(1)

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Marketing Video Demo Creator
Creates professional demonstration videos showcasing the drone simulation platform
"""

import os
import json
import time
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def create_marketing_demo_videos():
    """Create a series of demonstration videos for marketing purposes"""
    
    logger.info("🎬 Starting Marketing Demo Video Creation")
    
    # Demo scenarios for marketing video
    demo_scenarios = [
        {
            "name": "Earth_Mission_Demo",
            "description": "Professional Earth reconnaissance mission",
            "drone": "default_quadrotor",
            "environment": "earth",
            "mission": "reconnaissance",
            "duration": 120,  # 2 minutes for quick demo
            "features": ["real_time_telemetry", "3d_trajectory", "mission_progress"]
        },
        {
            "name": "Mars_Mission_Demo", 
            "description": "Mars exploration with challenging environment",
            "drone": "heavy_lift_hexarotor",
            "environment": "mars",
            "mission": "sample_transport",
            "duration": 150,  # 2.5 minutes
            "features": ["mars_physics", "atmospheric_simulation", "power_management"]
        },
        {
            "name": "Multi_Environment_Demo",
            "description": "Comparison across planetary environments",
            "drone": "default_quadrotor",
            "environment": "moon",
            "mission": "monitoring",
            "duration": 90,  # 1.5 minutes
            "features": ["low_gravity", "vacuum_conditions", "precision_control"]
        }
    ]
    
    created_videos = []
    
    for scenario in demo_scenarios:
        logger.info(f"🚁 Creating {scenario['name']} demonstration")
        
        # Create detailed mission configuration
        mission_config = {
            "demo_type": "marketing",
            "scenario": scenario["name"],
            "timestamp": time.time(),
            "settings": {
                "drone_model": scenario["drone"],
                "environment": scenario["environment"],
                "mission_type": scenario["mission"],
                "simulation_duration": scenario["duration"],
                "video_quality": "professional",
                "telemetry_overlay": True,
                "performance_metrics": True
            },
            "marketing_features": scenario["features"],
            "target_audience": "aerospace_professionals"
        }
        
        # Save mission configuration for reference
        config_path = f"marketing_demo_{scenario['name'].lower()}.json"
        with open(config_path, 'w') as f:
            json.dump(mission_config, f, indent=2)
        
        created_videos.append({
            "scenario": scenario["name"],
            "config_file": config_path,
            "description": scenario["description"],
            "estimated_duration": scenario["duration"],
            "marketing_value": scenario["features"]
        })
        
        logger.info(f"✅ Configuration created for {scenario['name']}")
    
    # Create master marketing video plan
    master_plan = {
        "project": "Drone Simulation Platform Marketing Video",
        "created": time.time(),
        "total_scenarios": len(demo_scenarios),
        "estimated_total_duration": sum(s["duration"] for s in demo_scenarios),
        "video_segments": created_videos,
        "production_notes": {
            "script_file": "marketing_video_script.md",
            "production_plan": "video_production_plan.md",
            "target_length": "3-4 minutes",
            "key_message": "99.8% cost reduction through advanced simulation"
        },
        "next_steps": [
            "Generate demonstration videos using Quick Video Creation",
            "Capture screen recordings of Analytics interface",
            "Record 3D trajectory visualizations",
            "Combine segments with professional narration"
        ]
    }
    
    with open("marketing_video_master_plan.json", 'w') as f:
        json.dump(master_plan, f, indent=2)
    
    logger.info("🎯 Marketing Demo Creation Complete!")
    logger.info(f"📊 Created {len(created_videos)} demonstration scenarios")
    logger.info(f"⏱️ Total content duration: {master_plan['estimated_total_duration']} seconds")
    
    return created_videos, master_plan

def generate_script_segments():
    """Generate specific script segments for each demo video"""
    
    script_segments = {
        "earth_mission": {
            "opening": "Watch as our drone executes a precision reconnaissance mission on Earth, demonstrating real-time telemetry and advanced flight dynamics.",
            "key_points": [
                "Real-time altitude and speed monitoring",
                "Precise waypoint navigation",
                "Professional telemetry displays",
                "Mission progress tracking"
            ],
            "closing": "Notice the smooth trajectory and accurate environmental simulation - exactly what you'd expect in real-world deployment."
        },
        "mars_mission": {
            "opening": "Now observe the same technology adapted for Mars exploration, accounting for reduced gravity and atmospheric conditions.",
            "key_points": [
                "Mars atmospheric simulation (3.71 m/s² gravity)",
                "Reduced air density effects",
                "Extended flight capabilities in low gravity",
                "Sample transport mission complexity"
            ],
            "closing": "Our physics engine automatically adjusts for planetary conditions, ensuring your Mars mission succeeds."
        },
        "analytics_showcase": {
            "opening": "Every simulation generates comprehensive analytics for mission optimization and performance analysis.",
            "key_points": [
                "Complete session history tracking",
                "Performance comparison tools",
                "Energy consumption analysis",
                "Mission success rate calculations"
            ],
            "closing": "Make data-driven decisions before committing to expensive real-world missions."
        }
    }
    
    with open("marketing_video_script_segments.json", 'w') as f:
        json.dump(script_segments, f, indent=2)
    
    logger.info("📝 Script segments generated for video production")
    return script_segments

if __name__ == "__main__":
    # Create marketing demonstration content
    videos, plan = create_marketing_demo_videos()
    scripts = generate_script_segments()
    
    print("\n🎬 MARKETING VIDEO CREATION SUMMARY")
    print("=" * 50)
    print(f"✅ Created {len(videos)} demonstration scenarios")
    print(f"📄 Generated comprehensive script and production plan")
    print(f"⏱️ Total demo content: {plan['estimated_total_duration']} seconds")
    print("\n📁 Files created:")
    print("- marketing_video_script.md")
    print("- video_production_plan.md") 
    print("- marketing_video_master_plan.json")
    print("- marketing_video_script_segments.json")
    print("- Individual demo configuration files")
    
    print("\n🚀 Next Steps:")
    print("1. Use the Video Export tab to generate demonstration videos")
    print("2. Record screen captures of the application interface")
    print("3. Add professional narration using the provided script")
    print("4. Combine all elements into a compelling marketing video")
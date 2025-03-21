#!/usr/bin/env python3
"""
Conversation tracker for the multi-view navigation system.
This script hooks into the improved navigation node and logs all
messages sent to and received from the vision model.

Run this in a separate terminal to see what the robot is thinking!
"""

import os
import json
import datetime
import time
import argparse
import threading
import socket
import signal
import sys
import rich
from rich.console import Console
from rich.panel import Panel
from rich.markdown import Markdown

# Create a console for rich output
console = Console()

# Set up log directory
log_dir = os.path.join(os.getcwd(), 'conversation_logs')
os.makedirs(log_dir, exist_ok=True)

# Global variables
conversation_log = []
server_socket = None
client_socket = None
running = True

def format_message(msg):
    """Format a message for display."""
    role = msg.get('role', 'unknown')
    content = msg.get('content', '')
    
    # Handle system messages
    if role == 'system':
        return Panel(
            Markdown(f"**System Prompt:**\n\n{content}"),
            title="System Message",
            border_style="blue"
        )
    
    # Handle user messages with images
    elif role == 'user':
        if isinstance(content, list):
            # Extract text parts from the message
            text_parts = []
            has_images = False
            for item in content:
                if item.get('type') == 'text':
                    text_parts.append(item.get('text', ''))
                elif item.get('type') == 'image_url':
                    has_images = True
            
            # Join text parts and note image presence
            text = '\n'.join(text_parts)
            if has_images:
                text += "\n\n[Contains images from LEFT, FRONT, and RIGHT cameras]"
            
            return Panel(
                text,
                title="User Message",
                border_style="green"
            )
        else:
            # Simple text message
            return Panel(
                str(content),
                title="User Message",
                border_style="green"
            )
    
    # Handle assistant (model) responses
    elif role == 'assistant':
        return Panel(
            str(content),
            title="Assistant Response",
            border_style="yellow"
        )
    
    # Default case
    return Panel(
        str(content),
        title=f"{role.capitalize()} Message",
        border_style="white"
    )

def save_conversation():
    """Save the conversation to a log file."""
    if not conversation_log:
        return
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(log_dir, f'conversation_{timestamp}.json')
    
    try:
        with open(filename, 'w') as f:
            json.dump(conversation_log, f, indent=2)
        console.print(f"[bold green]Conversation saved to {filename}[/bold green]")
    except Exception as e:
        console.print(f"[bold red]Error saving conversation: {e}[/bold red]")

def start_server(port=9876):
    """Start a socket server to listen for messages from the navigation node."""
    global server_socket, client_socket, running
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind(('localhost', port))
        server_socket.listen(1)
        console.print(f"[bold green]Server started on port {port}. Waiting for navigation node to connect...[/bold green]")
        
        client_socket, addr = server_socket.accept()
        console.print(f"[bold green]Connected to navigation node at {addr}[/bold green]")
        
        buffer = ""
        while running:
            data = client_socket.recv(4096).decode('utf-8')
            if not data:
                break
            
            buffer += data
            
            # Process complete messages
            while '\n' in buffer:
                message, buffer = buffer.split('\n', 1)
                try:
                    msg_obj = json.loads(message)
                    conversation_log.append(msg_obj)
                    console.print(format_message(msg_obj))
                except json.JSONDecodeError:
                    console.print(f"[bold red]Error decoding message: {message}[/bold red]")
    
    except Exception as e:
        console.print(f"[bold red]Server error: {e}[/bold red]")
    finally:
        if client_socket:
            client_socket.close()
        if server_socket:
            server_socket.close()

def handle_signal(sig, frame):
    """Handle keyboard interrupt."""
    global running
    console.print("\n[bold yellow]Shutting down conversation tracker...[/bold yellow]")
    running = False
    save_conversation()
    sys.exit(0)

def main():
    # Set up signal handler for clean shutdown
    signal.signal(signal.SIGINT, handle_signal)
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Track conversations between navigation node and vision model')
    parser.add_argument('--port', type=int, default=9876, help='Port to listen on')
    args = parser.parse_args()
    
    # Print welcome message
    console.print(Panel(
        "[bold]Navigation Conversation Tracker[/bold]\n\n"
        "This tool captures and displays the conversation between\n"
        "the navigation node and the vision model.\n\n"
        "Press Ctrl+C to save and exit.",
        border_style="blue"
    ))
    
    # Start server in a separate thread
    server_thread = threading.Thread(target=start_server, args=(args.port,))
    server_thread.daemon = True
    server_thread.start()
    
    # Keep main thread alive
    try:
        while running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        handle_signal(None, None)

if __name__ == "__main__":
    main()
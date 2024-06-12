update for this week 13
Non-blocking Accept: The server now periodically checks for new connections in a non-blocking manner.
Connection Management: Connection errors now lead to cleanup without blocking or immediate reconnection, enhancing robustness.
Socket Options: SO_REUSEADDR is set to avoid issues with socket re-binding after restarts.




The server now continuously tries to accept new connections in a loop.
It handles clients in a separate method, handle_client, which can be further expanded to manage multiple clients or handle client-specific data more gracefully.
It improves error handling around connection loss, logging more information when the connection is dropped.





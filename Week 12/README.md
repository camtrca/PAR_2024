## Packages

### topic_listener

In this package, only `processed_publisher` and `processed_subscriber` nodes are useful for cross domain communicate

Run subscriber in the follower domain or laptop domain first

```
ros2 run topic_listener processed_subscriber
```

Then run publisher in leader domain

```
ros2 run topic_listener processed_publisher
```

note: only tested on one topic, for more topics, my assumptions is when serializing the message combine all topic messages together seperated by our customised seperater.

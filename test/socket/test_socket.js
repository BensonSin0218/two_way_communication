import { io } from "socket.io-client";

const socket = io("http://127.0.0.1:1234")

socket.on("connect", () => {
    console.log("Connected!", socket.id)
    socket.emit("publish", { topic: "listener", message: { data: `I am ${socket.id}`}})

    setInterval(() => socket.emit("publish", {
        topic: "listener",
        message: { data: `${new Date().getSeconds()} @ ${socket.id}`}
    }), 1000)

    socket.emit("subscribe", { topic: "talker"})
    socket.on("talker", (message) => {
        console.log(new Date().getSeconds(), message.data)
    })

    setInterval(() => socket.emit("service", {
        service: "testing_service",
        request: { a: 1, b: 2 }
    },
    (res) => console.log("Service1:", res)), 5000)

    socket.on("disconnect", () => console.error("Disconnected"))
})
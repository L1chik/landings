#![cfg(feature = "stream")]

use opencv::core::Vector;

use std::{
    io::Result,
    pin::Pin,
    sync::{
        mpsc::{Sender as SSender, Receiver as SReceiver},
        Mutex,
    },
    task::{Context, Poll},
};

use actix_web::{web, Error, HttpServer, App, Responder, HttpResponse, middleware};
use actix_web::dev::ServerHandle;

use futures::Stream;

use tokio::sync::mpsc::{channel, Receiver, Sender};

pub async fn start(host: &str, port: i32, frames: SReceiver<Vector<u8>>, tx: SSender<ServerHandle>) -> Result<()> {
    let addr = format!("{}:{}", host, port);
    log::info!("Streaming at {}", addr);

    let broadcaster = web::Data::new(Mutex::new(Broadcaster::default()));
    Broadcaster::spawn_receiver(broadcaster.clone(), frames);

    let server = HttpServer::new(move || {
        App::new()
            .wrap(middleware::Logger::default())
            .app_data(broadcaster.clone())
            .configure(init_routes)
    })
        .bind(&addr).unwrap().workers(2)
        .run();

    let _ = tx.send(server.handle());

    server.await
}

async fn add_new_client(broadcaster: web::Data<Mutex<Broadcaster>>) -> impl Responder {
    let rx = broadcaster.lock().unwrap().add_client();
    HttpResponse::Ok()
        .append_header(("Cache-Control", "no-store, must-revalidate"))
        .append_header(("Pragma", "no-cache"))
        .append_header(("Expires", "0"))
        .append_header(("Connection", "close"))
        .append_header(("Content-Type", "multipart/x-mixed-replace;boundary=boundarydonotcross"))
        .streaming(rx)
}

async fn index() -> impl Responder {
    let content = include_str!("index.html");
    return HttpResponse::Ok().append_header(("Content-Type", "text/html")).body(content);
}
pub fn init_routes(cfg: &mut web::ServiceConfig) {
    cfg.route("/s1", web::get().to(add_new_client));
    cfg.route("/", web::get().to(index));
}

pub struct Broadcaster {
    clients: Vec<Sender<web::Bytes>>,
}

impl Broadcaster {
    pub fn default() -> Self {
        Broadcaster {
            clients: Vec::new(),
        }
    }

    pub fn add_client(&mut self) -> Client {
        let (tx, rx) = channel(1);
        self.clients.push(tx);
        Client(rx)
    }

    pub fn make_message_block(buffer: &Vector<u8>) -> Vec<u8> {
        let bfu8 = buffer.as_ref();
        let mut msg = format!(
            "--boundarydonotcross\r\
            Content-Length:{}\r\
            Content-Type:image/jpeg\r\n\r\n", bfu8.len()
        ).into_bytes();
        msg.extend(bfu8);
        msg
    }

    fn send_image(&mut self, msg: &[u8]) {
        let mut ok_clients = Vec::new();
        let msg = web::Bytes::from([msg].concat());
        for client in self.clients.iter() {
            if let Ok(()) = client.clone().try_send(msg.clone()) {
                ok_clients.push(client.clone());
            }
        }
        self.clients = ok_clients;
    }

    pub fn spawn_receiver(_self: web::Data<Mutex<Self>>, frames: SReceiver<Vector<u8>>) {
        tokio::spawn(async move {
            for received in frames {
                let msg = Broadcaster::make_message_block(&received);
                _self.lock().unwrap().send_image(&msg);
            }
        });
    }
}

pub struct Client(Receiver<web::Bytes>);

impl Stream for Client {
    type Item = std::result::Result<web::Bytes, Error>;
    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        match Pin::new(&mut self.0).poll_recv(cx) {
            Poll::Ready(Some(v)) => Poll::Ready(Some(Ok(v))),
            Poll::Ready(None) => Poll::Ready(None),
            Poll::Pending => Poll::Pending
        }
    }
}
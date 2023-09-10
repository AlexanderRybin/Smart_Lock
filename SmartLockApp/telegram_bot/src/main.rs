//t.me/ar_smart_lock_botа

use std::{error::Error, sync::Arc, time::Duration};

use proto::{JsonBellState, JsonLockCommand};
use telegram_token::BOT_TOKEN;

use teloxide::{prelude::*, types::Me, utils::command::BotCommands};
use tokio::{sync::Mutex, time::sleep};

#[derive(Clone)]
struct ShState(Arc<BellCommand>, Arc<LockCommand>, Arc<Mutex<Vec<ChatId>>>);

const HTTP_ADDR_BELL: &str = "http://127.0.0.1:3000/bell";
const HTTP_ADDR_LOCK: &str = "http://127.0.0.1:3000/lock";

#[tokio::main]
async fn main() {
    let bot = Bot::new(BOT_TOKEN);
    let bot_clone = bot.clone();

    let id_vec = Arc::new(Mutex::new(Vec::<ChatId>::new()));

    let bell_state = BellState(Mutex::new(false));
    let lock_command = Arc::new(LockCommand(Mutex::new(false)));
    let bell_command = Arc::new(BellCommand(Mutex::new(false)));

    let lock_command_clone = lock_command.clone();
    let bell_command_clone = bell_command.clone();
    let id_vec_clone = id_vec.clone();

    let sh_state = ShState(bell_command_clone, lock_command_clone, id_vec);

    let client = reqwest::Client::new();

    tokio::spawn(async move {
        loop {
            let bell_json = JsonBellState { bell: false };

            let bell_request = client.get(HTTP_ADDR_BELL).json(&bell_json).send().await;

            if let Ok(bell_response) = bell_request {
                if bell_response.status().is_success() {
                    match bell_response.json::<JsonBellState>().await {
                        Ok(bell) => {
                            if bell.bell {
                                println!("THE BELL IS RINGING!!!!");
                                bell_state.set().await;
                                for id in id_vec_clone.lock().await.iter() {
                                    bot_clone
                                        .send_message(*id, "🔔 THE BELL IS RINGING!!! 🔔")
                                        .await
                                        .unwrap();
                                }
                            } else if bell_state.get().await {
                                bell_state.reset().await;
                            }
                        }
                        Err(_) => {
                            println!("bell response parse error");
                            break;
                        }
                    }
                } else {
                    println!("bell response error");
                    break;
                }
            };

            if bell_command.get().await {
                bell_command.reset().await;
                let bell_json = JsonBellState { bell: false };

                let _ = client.post(HTTP_ADDR_BELL).json(&bell_json).send().await;
            }

            if lock_command.get().await {
                lock_command.reset().await;
                let lock_json = JsonLockCommand { lock: false };

                let _ = client.post(HTTP_ADDR_LOCK).json(&lock_json).send().await;
            }

            sleep(Duration::from_millis(1000)).await;
        }
    });

    let handler = dptree::entry().branch(Update::filter_message().endpoint(message_handler));

    Dispatcher::builder(bot, handler)
        // Pass the shared state to the handler as a dependency.
        .dependencies(dptree::deps![sh_state])
        .enable_ctrlc_handler()
        .build()
        .dispatch()
        .await;
}

#[derive(BotCommands)]
#[command(
    rename_rule = "lowercase",
    description = "These commands are supported:"
)]
enum Command {
    #[command(description = "Display this text")]
    Help,
    #[command(description = "Start receiving notifications")]
    Start,
    #[command(description = "Stop receiving notifications")]
    Stop,
    #[command(description = "Reset Bell")]
    ResetBell,
    #[command(description = "Open")]
    Open,
}

async fn message_handler(
    bot: Bot,
    state: ShState,
    msg: Message,
    me: Me,
) -> Result<(), Box<dyn Error + Send + Sync>> {
    if let Some(text) = msg.text() {
        match BotCommands::parse(text, me.username()) {
            Ok(Command::Help) => {
                bot.send_message(msg.chat.id, Command::descriptions().to_string())
                    .await?;
            }
            Ok(Command::Start) => {
                bot.send_message(msg.chat.id, Command::descriptions().to_string())
                    .await?;
                let mut id_vec = state.2.lock().await;
                id_vec.push(msg.chat.id);
            }
            Ok(Command::Stop) => {
                let mut id_vec = state.2.lock().await;
                id_vec.retain(|x| *x != msg.chat.id);
            }
            Ok(Command::ResetBell) => {
                state.0.set().await;
            }
            Ok(Command::Open) => {
                state.1.set().await;
            }

            Err(_) => {
                bot.send_message(msg.chat.id, Command::descriptions().to_string())
                    .await?;
            }
        }
    }

    Ok(())
}

#[derive(Default)]
struct BellState(Mutex<bool>);

impl BellState {
    async fn reset(&self) {
        let mut n = self.0.lock().await;
        *n = false;
    }

    async fn set(&self) {
        let mut n = self.0.lock().await;
        *n = true;
    }

    async fn get(&self) -> bool {
        let n = self.0.lock().await;
        *n
    }
}

#[derive(Default)]
struct BellCommand(Mutex<bool>);

impl BellCommand {
    async fn reset(&self) {
        let mut n = self.0.lock().await;
        *n = false;
    }

    async fn set(&self) {
        let mut n = self.0.lock().await;
        *n = true;
    }

    async fn get(&self) -> bool {
        let n = self.0.lock().await;
        *n
    }
}
#[derive(Default)]
struct LockCommand(Mutex<bool>);

impl LockCommand {
    async fn reset(&self) {
        let mut n = self.0.lock().await;
        *n = false;
    }

    async fn set(&self) {
        let mut n = self.0.lock().await;
        *n = true;
    }

    async fn get(&self) -> bool {
        let n = self.0.lock().await;
        *n
    }
}

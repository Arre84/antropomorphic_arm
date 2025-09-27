#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/controler_mode.hpp"

#include <memory>
#include <chrono>
#include <functional> // Necesario para std::bind y std::placeholders

using namespace std::chrono_literals;

class Processing : public rclcpp::Node {
public:
    Processing() : Node("motion_controller_node") // Nombre del nodo corregido para seguir convenciones
    {
        // 1. Creación del cliente para el servicio de control de modo.
        control_mode_client_ = this->create_client<custom_interfaces::srv::ControlerMode>("controler_service");

        // 2. Declaración del parámetro que será monitoreado.
        this->declare_parameter("Mode", true);

        // 3. Creación del manejador de eventos de parámetros.
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // 4. Definición del callback que se ejecutará cuando el parámetro "Mode" cambie.
        auto parameter_callback =
            [this](const rclcpp::Parameter & p) {
                // Verificamos si el servicio está disponible ANTES de intentar usarlo.
                // Esto evita bloquear el nodo si el servidor no está activo.
                if (!control_mode_client_->service_is_ready()) {
                    RCLCPP_ERROR(this->get_logger(), "El servicio 'controler_service' no está disponible. No se pudo enviar el cambio de modo.");
                    return;
                }

                // Creamos la petición del servicio.
                auto request = std::make_shared<custom_interfaces::srv::ControlerMode::Request>();
                
                // Usamos el valor del parámetro 'p' que activó el callback.
                // Esta es la forma correcta de obtener el nuevo valor.
                request->control_mode = p.as_bool();
                RCLCPP_INFO(this->get_logger(), "Parámetro 'Mode' cambiado a: %s. Enviando petición...", p.as_bool() ? "true" : "false");

                // Enviamos la petición de forma asíncrona.
                // El resultado se manejará en 'response_callback' cuando llegue.
                control_mode_client_->async_send_request(
                    request, std::bind(&Processing::response_callback, this, std::placeholders::_1));
            };

        // 5. Registramos el callback para el parámetro específico "Mode".
        param_handle_ = param_subscriber_->add_parameter_callback("Mode", parameter_callback);

        RCLCPP_INFO(this->get_logger(), "Nodo 'processing_node' iniciado y monitoreando el parámetro 'Mode'.");
    }

private:
    // ---- Miembros de la Clase ----
    // Es buena práctica declarar todos los miembros aquí.
    rclcpp::Client<custom_interfaces::srv::ControlerMode>::SharedPtr control_mode_client_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_handle_;

    /**
     * @brief Callback para manejar la respuesta del servicio.
     * @param future El resultado futuro que contiene la respuesta del servidor.
     */
    void response_callback(rclcpp::Client<custom_interfaces::srv::ControlerMode>::SharedFuture future)
    {
        // Obtenemos el resultado. Usamos try-catch por si la llamada al servicio falló.
        try {
            auto response = future.get();
            // Asumo que tu .srv tiene un booleano 'success' (no 'sucess').
            RCLCPP_INFO(this->get_logger(), "Respuesta del servicio: Modo de control actualizado a '%s'", response->current_mode.c_str());
            }
        catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Fallo al llamar al servicio 'controler_service': %s", e.what());
        }
    }
};

// --- Función main para ejecutar el nodo ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Processing>());
    rclcpp::shutdown();
    return 0;
}
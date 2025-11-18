import time
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor, Servo
import serial
import math

class WaterSamplingModule:
    def __init__(self):
        # Настройка пинов GPIO
        self.setup_gpio()
        
        # Инициализация компонентов
        self.setup_components()
        
        # Параметры системы
        self.target_volume = 100  # мл, целевой объем воды
        self.target_height = 50   # см, целевая высота над объектом
        self.current_volume = 0
        self.flow_pulse_count = 0
        
    def setup_gpio(self):
        """Настройка GPIO"""
        GPIO.setmode(GPIO.BCM)
        
        # Пины для компонентов
        self.pump_pin = 18        # Насос через реле
        self.flow_sensor_pin = 24 # Датчик расхода воды
        self.servo_pin = 17       # Сервопривод для подъема/опускания трубки
        self.ultrasonic_trigger = 23
        self.ultrasonic_echo = 24
        
        # Настройка пинов
        GPIO.setup(self.pump_pin, GPIO.OUT)
        GPIO.setup(self.flow_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def setup_components(self):
        """Инициализация компонентов"""
        # Ультразвуковой дальномер
        self.ultrasonic = DistanceSensor(echo=self.ultrasonic_echo, trigger=self.ultrasonic_trigger)
        
        # Сервопривод
        self.servo = Servo(self.servo_pin)
        
        # GPS модуль (симуляция)
        self.gps_connected = False
        self.setup_gps()
        
        # Насос выключен по умолчанию
        GPIO.output(self.pump_pin, GPIO.LOW)
        
        # Настройка прерывания для датчика расхода
        GPIO.add_event_detect(self.flow_sensor_pin, GPIO.FALLING, callback=self.flow_pulse_callback)
        
    def setup_gps(self):
        """Настройка GPS модуля"""
        try:
            self.gps_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)
            self.gps_connected = True
            print("GPS модуль инициализирован")
        except:
            print("Ошибка инициализации GPS модуля")
            self.gps_connected = False
    
    def flow_pulse_callback(self, channel):
        """Callback для подсчета импульсов датчика расхода"""
        self.flow_pulse_count += 1
        # YF-S201: ~450 импульсов на литр
        self.current_volume = (self.flow_pulse_count / 450.0) * 1000  # в мл
    
    def get_gps_coordinates(self):
        """Получение текущих GPS координат"""
        if not self.gps_connected:
            # Заглушка для тестирования
            return (55.7558, 37.6173)  # Координаты Москвы
        
        try:
            line = self.gps_serial.readline().decode('utf-8')
            if line.startswith('$GPGGA'):
                data = line.split(',')
                if data[2] and data[4]:  # Проверка наличия данных
                    lat = float(data[2][:2]) + float(data[2][2:])/60
                    lon = float(data[4][:3]) + float(data[4][3:])/60
                    if data[3] == 'S': lat = -lat
                    if data[5] == 'W': lon = -lon
                    return (lat, lon)
        except:
            pass
        
        return None
    
    def get_current_height(self):
        """Получение текущей высоты над водой"""
        try:
            distance = self.ultrasonic.distance * 100  # перевод в см
            return distance
        except:
            return None
    
    def adjust_height(self, target_height):
        """Выравнивание высоты над объектом"""
        print(f"Корректировка высоты до {target_height} см")
        
        current_height = self.get_current_height()
        if current_height is None:
            print("Ошибка получения высоты")
            return False
        
        height_tolerance = 2  # см, допустимое отклонение
        
        while abs(current_height - target_height) > height_tolerance:
            # Здесь должна быть логика управления высотой дрона
            # Для демонстрации просто ждем
            print(f"Текущая высота: {current_height:.1f} см")
            time.sleep(1)
            current_height = self.get_current_height()
            
            if current_height is None:
                print("Ошибка получения высоты")
                return False
        
        print(f"Высота достигнута: {current_height:.1f} см")
        return True
    
    def lower_sampling_tube(self):
        """Опускание трубки для забора воды"""
        print("Опускание трубки для забора воды")
        self.servo.min()  # Поворот сервопривода для опускания трубки
        time.sleep(2)
    
    def raise_sampling_tube(self):
        """Поднятие трубки после забора воды"""
        print("Поднятие трубки после забора воды")
        self.servo.max()  # Поворот сервопривода для поднятия трубки
        time.sleep(2)
    
    def start_pump(self):
        """Включение насоса"""
        print("Включение насоса")
        GPIO.output(self.pump_pin, GPIO.HIGH)
        self.flow_pulse_count = 0
        self.current_volume = 0
    
    def stop_pump(self):
        """Выключение насоса"""
        print("Выключение насоса")
        GPIO.output(self.pump_pin, GPIO.LOW)
    
    def collect_water_sample(self, target_volume):
        """Процесс сбора образца воды"""
        print(f"Начало сбора образца, целевой объем: {target_volume} мл")
        
        self.start_pump()
        start_time = time.time()
        
        while self.current_volume < target_volume:
            print(f"Собрано: {self.current_volume:.1f} мл из {target_volume} мл")
            time.sleep(0.5)
            
            # Защита от бесконечного цикла
            if time.time() - start_time > 60:  # Максимум 60 секунд
                print("Превышено время сбора образца")
                self.stop_pump()
                return False
        
        self.stop_pump()
        print(f"Образец собран: {self.current_volume:.1f} мл")
        return True
    
    def has_reached_target_coordinates(self, target_lat, target_lon, tolerance=0.0001):
        """Проверка достижения целевых координат"""
        current_coords = self.get_gps_coordinates()
        if current_coords is None:
            return False
        
        current_lat, current_lon = current_coords
        distance = math.sqrt((current_lat - target_lat)**2 + (current_lon - target_lon)**2)
        
        return distance <= tolerance
    
    def main_sampling_algorithm(self, target_lat, target_lon):
        """Основной алгоритм работы устройства"""
        print("=== ЗАПУСК АЛГОРИТМА ОТБОРА ПРОБ ===")
        
        # Шаг 1: Долететь до нужной координаты
        print("1. Навигация к целевым координатам...")
        while not self.has_reached_target_coordinates(target_lat, target_lon):
            print("Дрон в пути...")
            time.sleep(2)
        print("Целевые координаты достигнуты!")
        
        # Шаг 2: Проверка и корректировка высоты
        print("2. Проверка высоты...")
        current_height = self.get_current_height()
        
        if current_height is None:
            print("Ошибка: невозможно определить высоту")
            return False
        
        if abs(current_height - self.target_height) > 2:
            print("Выравнивание высоты...")
            if not self.adjust_height(self.target_height):
                print("Ошибка выравнивания высоты")
                return False
        else:
            print(f"Высота соответствует требованиям: {current_height:.1f} см")
        
        # Шаг 3: Опускание трубки и забор пробы
        print("3. Приступаем к отбору образцов...")
        self.lower_sampling_tube()
        
        if not self.collect_water_sample(self.target_volume):
            print("Ошибка сбора образца")
            self.raise_sampling_tube()
            return False
        
        # Шаг 4: Поднятие трубки и завершение
        self.raise_sampling_tube()
        
        print("=== ОТБОР ПРОБ УСПЕШНО ЗАВЕРШЕН ===")
        return True
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.stop_pump()
        GPIO.cleanup()
        if self.gps_connected:
            self.gps_serial.close()
        print("Ресурсы освобождены")

# Пример использования
if __name__ == "__main__":
    try:
        sampling_module = WaterSamplingModule()
        
        # Целевые координаты (пример)
        target_latitude = 55.7558
        target_longitude = 37.6173
        
        # Запуск основного алгоритма
        success = sampling_module.main_sampling_algorithm(target_latitude, target_longitude)
        
        if success:
            print("Миссия выполнена успешно!")
        else:
            print("Миссия завершена с ошибками!")
    
    except KeyboardInterrupt:
        print("Программа прервана пользователем")
    
    except Exception as e:
        print(f"Произошла ошибка: {e}")
    
    finally:
        if 'sampling_module' in locals():
            sampling_module.cleanup()
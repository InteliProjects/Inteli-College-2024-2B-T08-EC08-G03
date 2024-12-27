"use client";
import Navbar from "../components/Navbar";
import { columns } from "./columns"
import { DataTable } from "./data-table"
import { DatePickerWithRange } from "@/components/ui/date-picker";
import { Input } from "@/components/ui/input";
import axios from "axios";
import { DropdownMenu,
  DropdownMenuTrigger,
  DropdownMenuContent,
  DropdownMenuCheckboxItem

} from "@/components/ui/dropdown-menu"; 
import { Button } from "@/components/ui/button";
import { ChevronDown } from "lucide-react";
import { useState, useEffect } from "react";
import { DateRange } from "react-day-picker";



export default function Logs() {
  const [data, setData] = useState([]); 
  const [filteredData, setFilteredData] = useState([]); 
  const [isLoading, setIsLoading] = useState(true); 
  const [selectedStatus, setSelectedStatus] = useState<string[]>([]);

  const fetchLogs = async () => {
    console.log("Chamando a API de logs");

    try {
      const response = await axios.get("http://localhost:3000/logs/list", {
        headers: {
          "Content-Type": "application/json",
          Accept: "application/json",
        },
      });
      console.log("Resposta da API:", response.data);

      const formattedData = response.data.map((log: any) => ({
        date: (new Date(log.date.split("/").reverse().join(" "))).toISOString(),
        user: log.user,
        action: log.action,
        category: log.category,
        location: log.local, 
        status: log.status,
      }));
      console.log(formattedData);
      setData(formattedData); 
      setFilteredData(formattedData);
      
    } catch (error: any) {
      console.error(
        "Erro ao pegar os logs da aplicação:",
        error.response?.data || error.message
      );
    } finally {
      setIsLoading(false); 
    }
  };

  const handleDateRangeSelect = (range: DateRange | undefined) => {
    if (range?.from && range?.to) {
      const startDate = (range?.from).toISOString();
      const endDate = (range?.to).toISOString();
      const filtered = data.filter((log) => {
        const logDate = (new Date(log.date)).toISOString();
        return logDate >= startDate && logDate <= endDate;
      });
      setFilteredData(filtered);
    } else {
      setFilteredData(data);
    }
  };

  const filterByStatus = () => {
    if (selectedStatus.length > 0) {
      const filtered = data.filter((log) =>
        selectedStatus.includes(log.status)
      );
      setFilteredData(filtered);
    } else {
      setFilteredData(data);
    }
  };


  useEffect(() => {
    fetchLogs();
  }, []);

  useEffect(() => {
    filterByStatus();
  }, [selectedStatus, data]);

  return (
    <div>
      <Navbar />
      <div className="p-5">
        <div className="text-black text-center py-3 flex flex-column gap-5 ">
          <DatePickerWithRange 
            onSelect={handleDateRangeSelect}
          />
          <Input
            className="w-48"
            placeholder="Filtrar por usuário"
            onChange={(e) => {
              const value = e.target.value.toLowerCase();
              if (value === "") {
          setFilteredData(data);
              } else {
            const filtered = data.filter((log) =>
            log.user.toLowerCase().startsWith(value)
            );
          setFilteredData(filtered);
              }
            }}
          />
          <Input
            className="w-48"
            placeholder="Filtrar por ação"
            onChange={(e) => {
              const value = e.target.value.toLowerCase();
              if (value === "") {
          setFilteredData(data);
              } else {
          const filtered = data.filter((log) =>
            log.action.toLowerCase().startsWith(value)
            );
          setFilteredData(filtered);
              }
            }}
          />
          <Input
            className="w-48"
            placeholder="Filtrar por categoria"
            onChange={(e) => {
              const value = e.target.value.toLowerCase();
              if (value === "") {
          setFilteredData(data);
              } else {
          const filtered = data.filter((log) =>
            log.category.toLowerCase().startsWith(value)
            );
          setFilteredData(filtered);
              }
            }}
          />
          <Input
            className="w-48"
            placeholder="Filtrar por destino"
            onChange={(e) => {
              const value = e.target.value.toLowerCase();
              if (value === "") {
          setFilteredData(data);
              } else {
          const filtered = data.filter((log) =>
            log.location.toLowerCase().startsWith(value)
            );
          setFilteredData(filtered);
              }
            }}
          />
            <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="outline" className="ml-auto relative z-10 w-[160px]">
                Status <ChevronDown />
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end" className="relative z-20 w-[160px]">
              {Array.from(new Set(data.map((log) => log.status))).map(
                (status) => (
                  <DropdownMenuCheckboxItem
                    key={status}
                    className="capitalize"
                    checked={selectedStatus.includes(status)}
                    onCheckedChange={(checked) => {
                      setSelectedStatus((prevSelectedStatus) =>
                        checked
                          ? [...prevSelectedStatus, status]
                          : prevSelectedStatus.filter(
                              (item) => item !== status
                            )
                      );
                    }}
                  >
                    {status}
                  </DropdownMenuCheckboxItem>
                )
              )}
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
          <DataTable columns={columns} data={filteredData} 
          className="table-fixed"/>
      </div>
    </div>
  );
}

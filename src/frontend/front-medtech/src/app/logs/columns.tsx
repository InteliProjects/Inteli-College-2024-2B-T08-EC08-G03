"use client";

import { ColumnDef } from "@tanstack/react-table";
import { Button } from "@/components/ui/button";
import { ArrowUpDown } from "lucide-react";

export type Register = {
  date: string;
  user: string;
  action: string;
  category: string;
  location: string;
  status: "Preparando" | "Finalizado" | "Robô indo pegar pedido" | "Entregando";
};

export const columns: ColumnDef<Register>[] = [
  {
    accessorKey: "date",
    header: ({ column }) => {
      return (
        <div className="justify-between">
        Data
        <Button
          variant="ghost"
          className="hover:bg-[#7DD9B1]"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
        </div>
      )
    },
  },

  {
    accessorKey: "user",
    header: ({ column }) => {
      return (
        <div className="justify-between">
        Usuário 
        <Button
          variant="ghost"
          className="hover:bg-[#7DD9B1]"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
        </div>
      )
    },
  },
  {
    accessorKey: "action",
    header: ({ column }) => {
      return (
        <div className="justify-between">
        Ação
        <Button
          variant="ghost"
          className="hover:bg-[#7DD9B1]"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
        </div>
      )
    },
  },
  {
    accessorKey: "category",
    header: ({ column }) => {
      return (
        <div className="justify-between">
        Categoria
        <Button
          variant="ghost"
          className="hover:bg-[#7DD9B1]"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
        </div>
      )
    },
  },

  {
    accessorKey: "location",
    header: ({ column }) => {
      return (
        <div className="justify-between">
        Destino
        <Button
          variant="ghost"
          className="hover:bg-[#7DD9B1]"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
        </div>
      )
    },
  },
  {
    accessorKey: "status",
    header: ({ column }) => {
      return (
        <div className="justify-between">
        Status
        <Button
          variant="ghost"
          className="hover:bg-[#7DD9B1]"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
        </div>
      )
    },
  },
];
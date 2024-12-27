import { Controller, Get, Post, Body, Param, Put, Delete } from '@nestjs/common';
import { User } from '../interfaces/user.interface';
import { CreateUser, UpdateUser } from 'src/dto/user.dto';
import { ApiOperation, ApiResponse } from '@nestjs/swagger';
import { UsersService } from '@services/user.service';

@Controller('users')
export class UsersController {
  constructor(private readonly usersService: UsersService) {}

  @Post()
  @ApiOperation({ summary: 'Criar um novo usuário' })
  @ApiResponse({
    status: 201,
    description: 'Usuário criado com sucesso.',
  })
  async createUser(@Body() createUserDto: CreateUser) {
    return this.usersService.createUser(createUserDto);
  }

  @Get()
  @ApiOperation({ summary: 'Listar todos os usuários' })
  @ApiResponse({
    status: 200,
    description: 'Usuários listados com sucesso.',
  })
  async listUsers(): Promise<User[]> {
    return this.usersService.listUsers();
  }

  @Get(':id')
  @ApiOperation({ summary: 'Buscar usuário pelo ID' })
  @ApiResponse({
    status: 200,
    description: 'Usuário encontrado.',
  })
  async getUser(@Param('id') id: number): Promise<User> {
    return this.usersService.getUser(id);
  }

  @Put(':id')
  @ApiOperation({ summary: 'Atualizar informações do usuário' })
  @ApiResponse({
    status: 200,
    description: 'Usuário atualizado com sucesso.',
  })
  async updateUser(@Param('id') id: number, @Body() updateUserDto: UpdateUser) {
    return this.usersService.updateUser(id, updateUserDto);
  }

  @Delete(':id')
  @ApiOperation({ summary: 'Deletar usuário' })
  @ApiResponse({
    status: 200,
    description: 'Usuário deletado com sucesso.',
  })
  async deleteUser(@Param('id') id: number) {
    return this.usersService.deleteUser(id);
  }
}
